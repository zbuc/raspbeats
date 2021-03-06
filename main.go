package main

import (
	"github.com/gordonklaus/portaudio"
	"github.com/nsf/termbox-go"
	"github.com/pelletier/go-toml"
	"github.com/stianeikeland/go-rpio"
	ring "github.com/zfjagann/golang-ring"

	// "bytes"
	"encoding/binary"
	"fmt"
	"io"
	"io/ioutil"
	"log"
	"math"
	"os"
	"os/exec"
	"os/signal"
	"strconv"
	"sync"
	"time"
)

const (
	LOW_PASS = iota
	HIGH_PASS
	BAND_PASS
	BAND_STOP
)

// The frame size in samples/points.
// 344 is 1/128 of a second at 44.1kHz
// This affects the ticker schedule as well as the latency of input
// processing. I.e. any changes to the mixer are audible on a per-frame
// basis (unless we adpot a point-based ring buffer instead)
const FRAME_SIZE = 344

// const FRAME_SIZE = 1378
// const FRAME_SIZE = 689

const SAMPLE_RATE = 44100

type Looper interface {
	Start()
	Stop()
}

type Loop struct {
}

var maxSize int16 = 0

func GetOutSamples(audio *io.Reader, sampleRate int, numChans int) []AudioFrame {
	outSamples := make([]AudioFrame, 0)
	if numChans == 2 {
		log.Fatalln("Stereo unsupported. committing suicide because i don't feel like implementing this yet.")
		for remaining := sampleRate; remaining > 0; remaining -= FRAME_SIZE {
			out := make([]int16, FRAME_SIZE)
			// slice out to be only the length of what we have left to return
			if len(out) > remaining {
				out = out[:remaining]
			}
			err := binary.Read(*audio, binary.BigEndian, out)
			if err == io.EOF {
				log.Printf("io.EOF\n")
				break
			}
			chk(err)
			for _, r := range out {
				if math.Abs(float64(r)) > math.Abs(float64(maxSize)) {
					maxSize = int16(r)
				}
			}
			outSamples = append(outSamples, *ConvertPCMFrameToFloat(out))
		}

	} else {
		for remaining := sampleRate; remaining > 0; remaining -= FRAME_SIZE {
			out := make([]int16, FRAME_SIZE)
			if len(out) > remaining {
				out = out[:remaining]
			}
			err := binary.Read(*audio, binary.BigEndian, out)
			if err == io.EOF {
				log.Printf("io.EOF\n")
				break
			}
			chk(err)
			for _, r := range out {
				if math.Abs(float64(r)) > math.Abs(float64(maxSize)) {
					maxSize = int16(r)
				}
			}
			outSamples = append(outSamples, *ConvertPCMFrameToFloat(out))
		}
	}

	return outSamples
}

func applyMix(a float32, b float32) float32 {
	// if a == 0 {
	// 	return b
	// }

	// if b == 0 {
	// 	return a
	// }

	return a + b
	// return (1.0 / math.Sqrt2) * float32(a+b)
}

// Mixes addedFrame * addedFrameVolumePercentage into baseFrame, modifying it in place
func MixFrames(baseFrame *AudioFrame, addedFrame *AudioFrame, addedFrameVolumePercentage float32) {
	var longerFrame *AudioFrame
	var shorterFrame *AudioFrame
	if len(*baseFrame) > len(*addedFrame) {
		longerFrame = baseFrame
		shorterFrame = addedFrame
	} else {
		longerFrame = addedFrame
		shorterFrame = baseFrame
	}

	i := 0
	var p float32
	for i, p = range *longerFrame {
		if len(*shorterFrame) > i {
			if longerFrame == addedFrame {
				(*baseFrame)[i] = applyMix(addedFrameVolumePercentage*p, (*shorterFrame)[i])
			} else {
				(*baseFrame)[i] = applyMix(p, addedFrameVolumePercentage*(*shorterFrame)[i])
			}
		} else {
			if longerFrame == addedFrame {
				(*baseFrame)[i] = addedFrameVolumePercentage * p
			} else {
				(*baseFrame)[i] = p
			}
		}
	}
}

func GetAudio(f *os.File) (io.Reader, int, int) {
	id, data, err := readChunk(f)
	chk(err)
	if id.String() != "FORM" {
		log.Println("bad file format")
		log.Println(id.String())
		return nil, 0, 0
	}
	_, err = data.Read(id[:])
	chk(err)
	if id.String() != "AIFF" {
		log.Println("bad file format")
		log.Println(id.String())
		return nil, 0, 0
	}

	var c commonChunk
	var audio io.Reader
	for {
		id, chunk, err := readChunk(data)
		if err == io.EOF {
			break
		}
		chk(err)
		switch id.String() {
		case "COMM":
			chk(binary.Read(chunk, binary.BigEndian, &c))
			log.Printf("Read file format info: %+v\n", c)
		case "SSND":
			chunk.Seek(8, 1) //ignore offset and block
			audio = chunk
		default:
			log.Printf("ignoring unknown chunk '%s'\n", id)
		}
	}

	log.Printf("1: %+v\n", audio)
	return audio, int(c.NumSamples), int(c.NumChans)
}

type AudioPoint float32

func playEmptyAudioFrame(audioFrameBuffer *AudioFrameRingBuffer, out *[]float32, stream *portaudio.Stream) {
	audioFrameBuffer.m.Lock()
	defer audioFrameBuffer.m.Unlock()

	nothing := make([]float32, FRAME_SIZE)
	*out = ([]float32)(nothing)
	err := stream.Write()
	if err != nil {
		log.Println(err)
	}
}

func playAudioFrame(audioFrameBuffer *AudioFrameRingBuffer, out *[]float32, stream *portaudio.Stream) {
	audioFrameBuffer.m.Lock()
	defer audioFrameBuffer.m.Unlock()

	mixedAudio := audioFrameBuffer.GetFrame()

	*out = ([]float32)(mixedAudio)
	err := stream.Write()
	if err != nil {
		log.Println(err)
	}
}

type Sample struct {
	SampleRate int
	Audio      *io.Reader
	OutSamples *[]AudioFrame
	Name       string
}

type Track struct {
	Sample   *Sample
	Volume   int
	Soundset int
}

func tbLine(msg string) {
	const coldef = termbox.ColorDefault
	tbprint(0, lineOffset, termbox.ColorMagenta, coldef, msg)
	lineOffset++
}

func tbprint(x, y int, fg, bg termbox.Attribute, msg string) {
	for _, c := range msg {
		termbox.SetCell(x, y, c, fg, bg)
		x++
	}
}

type ScreenContext struct {
	Tracks           *[]Track
	SelectedIndex    int
	Filter           *Filter
	SelectedSoundset int
	MaxSoundset      int
}

type Mixer struct {
	Tracks       *[]Track
	MixedAudio   *[]AudioFrame
	CurrentFrame int
}

func redrawAll(context *ScreenContext) {
	const coldef = termbox.ColorDefault
	termbox.Clear(coldef, coldef)
	tbprint(0, 0, termbox.ColorMagenta, coldef, "Press 'ctrl+c' to quit")
	tbprint(0, 1, termbox.ColorMagenta, coldef, "Arrow keys to navigate, 'u' for volume up, 'd' for volume down. 'm' for max, 'c' for 0.")
	tbprint(0, 2, termbox.ColorMagenta, coldef, "'t'/'r' for filter cutoff, 'g'/'h' for resonance")

	offset := 4
	for i, track := range *context.Tracks {
		if i == context.SelectedIndex {
			tbprint(0, i+offset, coldef, coldef, fmt.Sprintf("Name: %-50s | Volume: %-10d >", track.Sample.Name, track.Volume))
		} else {
			tbprint(0, i+offset, coldef, coldef, fmt.Sprintf("Name: %-50s | Volume: %-10d", track.Sample.Name, track.Volume))
		}
	}

	offset += len(*context.Tracks) + 1
	tbprint(0, offset, coldef, coldef, fmt.Sprintf("Cutoff %f", context.Filter.Cutoff))
	offset++
	tbprint(0, offset, coldef, coldef, fmt.Sprintf("Resonance %f", context.Filter.Resonance))
	// for
	// 	tbprint(0, 2, coldef, coldef,
	// 		fmt.Sprintf("EventKey: k: %d, c: %c, mod: %s", curev.Key, curev.Ch, mod_str(curev.Mod)))
	termbox.Flush()
}

func zeroAudio(audio *AudioFrame) {
	for i, _ := range *audio {
		(*audio)[i] = 0
	}
}

var lineOffset = 10

type AudioFrame []float32

// http://www.martin-finke.de/blog/articles/audio-plugins-013-filter/
type Filter struct {
	buf0      float32
	buf1      float32
	Mode      int
	Cutoff    float32
	Resonance float32
}

// digital filtering info: http://www.drdobbs.com/parallel/digital-filtering-and-oversampling/184404066
func (f *Filter) Process(inputSample float32) float32 {
	//set feedback amount given f and q between 0 and 1
	fb := f.Resonance + f.Resonance/(1.0-f.Cutoff)

	//for each sample...
	f.buf0 = f.buf0 + f.Cutoff*(inputSample-f.buf0+fb*(f.buf0-f.buf1))
	f.buf1 = f.buf1 + f.Cutoff*(f.buf0-f.buf1)
	return f.buf1
}

func applyFilter(frame *AudioFrame, f *Filter) {
	for i, p := range *frame {
		(*frame)[i] = f.Process(p)
	}

	return
}

// func AddHeadroomToMixedFrames(mixedFrame *AudioFrame, trackCount int16) {
// 	for i, p := range *mixedFrame {
// 		(*mixedFrame)[i] = p / trackCount
// 	}
// }

// Grab next frame for all samples, perform mixing, add to buffer
func runMixer(mixer *Mixer, audioFrameBuffer *AudioFrameRingBuffer, maxFrameLength int, f *Filter, selectedSoundset int) {
	// log.Println("Creating mixed audio frames...")

	mixedFrame := make(AudioFrame, FRAME_SIZE)
	zeroAudio(&mixedFrame)

	// wrap around at the end -- should we just use a ring buffer?
	if mixer.CurrentFrame >= maxFrameLength {
		mixer.CurrentFrame = 0
	}

	// mix every track together
	for _, track := range *mixer.Tracks {
		if track.Soundset != selectedSoundset {
			continue
		}
		sample := track.Sample
		volumePercentage := float32(track.Volume) / 100.0

		if mixer.CurrentFrame < len(*sample.OutSamples) {
			MixFrames(&mixedFrame, &(*sample.OutSamples)[mixer.CurrentFrame], volumePercentage)
		}
	}

	// return (1.0 / math.Sqrt2) * float32(a+b)

	// then we have to divide each mixed output sample by the total number of samples playing to create
	// headroom
	for i, p := range mixedFrame {
		mixedFrame[i] = (1.0 / math.Sqrt2) * p
	}

	// floatFrame := ConvertPCMFrameToFloat(mixedFrame)

	applyFilter(&mixedFrame, f)

	// XXX memory inefficient
	// mixedFrame = *ConvertFloatFrameToPCMFrame(*floatFrame)

	enqueueMixedAudioFrame(audioFrameBuffer, &mixedFrame)

	mixer.CurrentFrame++
}

func enqueueMixedAudioFrame(audioFrameBuffer *AudioFrameRingBuffer, mixedFrame *AudioFrame) {
	audioFrameBuffer.AddFrame(*mixedFrame)
}

func SetupLogger() *os.File {
	f, err := os.OpenFile("dev.log", os.O_RDWR|os.O_CREATE|os.O_APPEND, 0666)
	if err != nil {
		panic(fmt.Sprintf("error opening file: %v", err))
	}

	log.SetOutput(f)
	return f
}

type AudioPointRingBuffer struct {
	r ring.Ring
	m sync.Mutex
}

type AudioFrameRingBuffer struct {
	r        ring.Ring
	m        sync.Mutex
	produced int
}

func (b *AudioFrameRingBuffer) AddFrame(frame AudioFrame) {
	b.m.Lock()
	defer b.m.Unlock()

	b.produced++

	// was passing pointers here but maybe not? don't *want* to realloc...
	b.r.Enqueue(frame)
}

func (b *AudioFrameRingBuffer) GetFrame() AudioFrame {
	b.produced--

	frame := b.r.Dequeue()
	if frame == nil {
		return nil
	}
	return frame.(AudioFrame)
}

func (b *AudioFrameRingBuffer) Produced() int {
	b.m.Lock()
	defer b.m.Unlock()

	return b.produced
}

func (b *AudioFrameRingBuffer) Capacity() int {
	b.m.Lock()
	defer b.m.Unlock()

	return b.r.Capacity()
}

func ConvertFloatFrameToPCMFrame(frameFloat []float32) *[]int16 {
	out := make([]int16, len(frameFloat))

	for i, f := range frameFloat {
		out[i] = int16(f * 32768)

		if out[i] > 32767 {
			out[i] = 32767
		}

		if out[i] < -32767 {
			out[i] = -32767
		}
	}

	return &out
}

func ConvertPCMFrameToFloat(framePCM []int16) *[]float32 {
	out := make([]float32, len(framePCM))

	for i, f := range framePCM {
		// log.Printf("dividing %f by 32768.0\n", float64(f))
		out[i] = float32(f) / 32768.0

		if out[i] > 1 {
			out[i] = 1
		}

		if out[i] < -1 {
			out[i] = -1
		}
	}

	return &out
}

type SampleSet map[string][]Sample

type GPIOBehavior struct {
	LastTriggered time.Time
	Behavior      string
	Pin           int
}

// maps a GPIO pin # to its associated behavior
type GPIOPinBehaviors map[rpio.Pin]GPIOBehavior

func initGPIO(conf *toml.TomlTree) GPIOPinBehaviors {
	err := rpio.Open()

	if err != nil {
		log.Println("Error initializing GPIO", err)
	}

	// just hackily hardcoding these for now, should move to conf file
	pins := make(GPIOPinBehaviors)
	for k, v := range conf.ToMap() {
		sk, err := strconv.Atoi(k)
		if err != nil {
			panic(err)
		}

		pin := rpio.Pin(sk)
		pins[pin] = GPIOBehavior{
			Behavior: v.(string),
			Pin:      sk,
		}

		// configure pin with PullUp and input mode
		pin.Input()
		pin.PullUp()
	}

	return pins
}

var lastTriggeredBehaviors = make(map[string]time.Time)

func switchCutoff(context *ScreenContext) {
	playbackmtx.Lock()
	defer playbackmtx.Unlock()

	if (*context.Filter).Cutoff >= 1 {
		(*context.Filter).Cutoff = 0.39
	} else {
		(*context.Filter).Cutoff += 0.15
	}
}

func triggerBehavior(behavior GPIOBehavior, context *ScreenContext) {
	// one quarter second delay between changes
	longAgo := time.Now().Add(1 * time.Second)
	then := time.Now().Add(-250 * time.Millisecond)
	if lastTriggeredBehaviors[behavior.Behavior].After(then) {
		return
	}

	if behavior.Behavior == "nextsoundset" {
		if lastTriggeredBehaviors["switchCutoff"].After(longAgo) {
			// these two are glitchy on the board
			return
		}

		if (*context).SelectedSoundset < (*context).MaxSoundset {
			(*context).SelectedSoundset++
		} else {
			// wrap around
			(*context).SelectedSoundset = 0
		}

		log.Printf("Selected soundset %d\n", (*context).SelectedSoundset)
	}

	if behavior.Behavior == "prevsoundset" {
		if lastTriggeredBehaviors["switchCutoff"].After(longAgo) {
			// these two are glitchy on the board
			return
		}

		if (*context).SelectedSoundset > 1 {
			(*context).SelectedSoundset--
		} else {
			// wrap around
			(*context).SelectedSoundset = (*context).MaxSoundset
		}

		log.Printf("Selected soundset %d\n", (*context).SelectedSoundset)
	}

	if behavior.Behavior == "toggleTrack" && behavior.Pin != 0 {
		// pin 0 is messed up right now, hack
		log.Printf("toggling %d\n", behavior.Pin)
		toggleVolume(behavior.Pin, context)
	}

	if behavior.Behavior == "switchCutoff" {
		if lastTriggeredBehaviors["nextsoundset"].After(longAgo) || lastTriggeredBehaviors["prevsoundset"].After(longAgo) {
			fmt.Println("Yawn")
			// these two are glitchy on the board
			return
		}
		switchCutoff(context)
	}

	lastTriggeredBehaviors[behavior.Behavior] = time.Now()
}

func playTrack(track Track, out *[]float32, stream *portaudio.Stream) {
	for _, frame := range *track.Sample.OutSamples {
		(*out) = frame
		_ = stream.Write()
	}
}

func playIntro() {
	fileName := "assets/samples/intro.aiff"

	cmd := exec.Command("aplay", "-fS16_BE", "-r44100", "-c1", fileName)

	cmd.Start()
	cmd.Wait()
}

var playbackmtx sync.Mutex
var playbackAllowed bool = false

func toggleVolume(trackNum int, context *ScreenContext) {
	playbackmtx.Lock()
	defer playbackmtx.Unlock()

	i := 0
	for j, track := range *context.Tracks {
		if track.Soundset != context.SelectedSoundset {
			continue
		}

		i++
		if i == trackNum {
			if (*context.Tracks)[j].Volume > 50 {
				(*context.Tracks)[j].Volume = 0
			} else {
				(*context.Tracks)[j].Volume = 100
			}
		}
	}

}

func isPlaybackAllowed() bool {
	playbackmtx.Lock()
	defer playbackmtx.Unlock()
	return playbackAllowed
}

func allowPlayback() {
	playbackmtx.Lock()
	defer playbackmtx.Unlock()
	log.Println("Allowibg playback")
	playbackAllowed = true
}

func restartExperience() {
	playbackmtx.Lock()
	defer playbackmtx.Unlock()
	log.Println("Restarting experience for the next person")
	playbackAllowed = false
}

func main() {
	loggerFile := SetupLogger()
	defer loggerFile.Close()

	log.SetOutput(ioutil.Discard)

	timeToSendFrame := time.Second / (SAMPLE_RATE / FRAME_SIZE)
	fmt.Printf("%s second timer should do\n", timeToSendFrame)

	if len(os.Args) != 2 {
		log.Println("missing required argument: soundsets.toml")
		return
	}

	log.Println("Playing.  Press Ctrl-C to stop.")

	tracks := []Track{}
	screenContext := &ScreenContext{Tracks: &tracks}

	confFileName := os.Args[1]
	sampleSetConf, err := toml.LoadFile(confFileName)

	portaudio.Initialize()
	defer portaudio.Terminate()

	out := make([]float32, FRAME_SIZE)
	devs, err := portaudio.Devices()
	if err != nil {
		log.Printf("error enumerating devices: %v\n", err)
		return
	}
	for x, g := range devs {
		log.Printf("%d: %+v\n", x, g)
	}
	i := 2
	if i < 0 {
		return
	}

	p := portaudio.HighLatencyParameters(nil, devs[i])
	p.SampleRate = 44100.0
	p.FramesPerBuffer = len(out)
	p.Output.Channels = 1
	stream, err := portaudio.OpenStream(p, &out)

	chk(err)
	defer stream.Close()
	chk(stream.Start())
	defer stream.Stop()

	log.Printf("Selected Output Device Info: %+v\n", devs[i])

	log.Printf("Stream info: %+v\n", stream.Info())

	pins := initGPIO(sampleSetConf.Get("GPIOConfigs").(*toml.TomlTree))
	log.Printf("Got pin config: %v\n", pins)

	GPIODoneChan := make(chan bool)
	go func(pins GPIOPinBehaviors, doneChan chan bool) {
		quit := false
		go func() {
			quit = <-doneChan
			log.Println("Received done message on GPIO done chan")
		}()

		// state for whether the phone is lifted or not
		phoneLifted := false

		// while the phone is not lifted, we wait
		// when the phone is lifted, play the intro, then begin the main loop
		// when the phone is put back down, wait for it to be lifted again and repeat

		// check every 1/8 second
		timeToCheck := time.Millisecond * 125

		// hacking this in here because i don't have time to do it well
		// combos := [][]int{{4, 27}, {4, 6}, {4, 19}, {17, 27}, {17, 6}, {17, 19}, {27, 5}, {22, 13}, {22, 19}, {5, 6}, {5, 19}, {26, 18}, {12, 16}}
		combos := [][]int{{4, 27}, {4, 17}, {4, 19}, {17, 27}, {17, 6}, {17, 19}, {27, 5}, {22, 13}, {22, 19}, {5, 6}, {5, 19}, {26, 18}, {12, 16}}
		comboPins := make([][]rpio.Pin, len(combos))
		for i, pair := range combos {
			comboPins[i] = []rpio.Pin{rpio.Pin(pair[0]), rpio.Pin(pair[1])}
		}

		ticker := time.NewTicker(timeToCheck)
		for _ = range ticker.C {
			for i, pair := range comboPins {
				// we need to fire some electrons through pair[0]
				// and receive at pair[1]
				pair[0].Output()
				pair[0].Write(1)
				pair[1].Input()
				pair[1].PullDown()

				// these read positive
				if !phoneLifted && i == 12 {
					if pair[1].Read() == 1 {
						// this is the headset lift
						playIntro()
						allowPlayback()
						phoneLifted = true
					}
				} else if phoneLifted && i == 12 {
					// putting the phone back down
					if pair[1].Read() == 0 {
						// they put it back
						restartExperience()
						phoneLifted = false
					}
				} else if pair[1].Read() == 1 {
					if i == 0 {
						triggerBehavior(GPIOBehavior{Behavior: "nextsoundset", Pin: 0}, screenContext)
					} else if i == 2 {
						triggerBehavior(GPIOBehavior{Behavior: "switchCutoff", Pin: 2}, screenContext)
					} else if i == 1 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 0}, screenContext)
					} else if i == 3 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 1}, screenContext)
					} else if i == 4 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 2}, screenContext)
					} else if i == 5 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 3}, screenContext)
					} else if i == 6 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 4}, screenContext)
					} else if i == 7 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 5}, screenContext)
					} else if i == 8 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 6}, screenContext)
					} else if i == 9 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 7}, screenContext)
					} else if i == 10 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 8}, screenContext)
					} else if i == 11 {
						triggerBehavior(GPIOBehavior{Behavior: "toggleTrack", Pin: 9}, screenContext)
					}
				}

				pair[0].Write(0)
			}

			// for pin, behavior := range pins {
			// 	if behavior.Pin == 6 {
			// 		if pin.Read() == 0 {
			// 			// allow playback to continue
			// 			allowPlayback()
			// 		} else {
			// 			restartExperience()
			// 		}
			// 	} else {
			// 		if pin.Read() == 0 {
			// 			triggerBehavior(behavior, screenContext)
			// 		}
			// 	}
			// }

			if quit {
				log.Printf("Quitting ticker")
				break
			}
		}
	}(pins, GPIODoneChan)

	part2 := sampleSetConf.Get("SampleSetConfigs").(*toml.TomlTree)

	(*screenContext).MaxSoundset = len(part2.Keys()) - 1
	for soundSet, sampleSetName := range part2.Keys() {
		log.Println(sampleSetName)
		fileNames := part2.Get(sampleSetName).([]*toml.TomlTree)
		for _, fileNameStruct := range fileNames {
			fileNameMap := fileNameStruct.ToMap()
			fileNamesIfaces := []interface{}(fileNameMap["fileNames"].([]interface{}))

			for _, fileName := range fileNamesIfaces {
				f, err := os.Open(fileName.(string))
				chk(err)
				defer f.Close()
				audio, sampleRate, numChans := GetAudio(f)
				outSamples := GetOutSamples(&audio, sampleRate, numChans)
				sample := &Sample{
					SampleRate: sampleRate,
					Audio:      &audio,
					OutSamples: &outSamples,
					Name:       fileName.(string),
				}
				volume := 100
				track := Track{
					Sample:   sample,
					Volume:   volume,
					Soundset: soundSet,
				}
				tracks = append(tracks, track)
			}
		}
	}

	// Should memoize some of these repeated length computations for efficiency
	log.Println("Step 1: Normalizing tracks to closest subdivision of longest track...")
	longestTrackLength := 0
	var longestSample *Sample
	for _, track := range tracks {
		trackLength := 0
		for _, sampleFrame := range *track.Sample.OutSamples {
			trackLength += len(sampleFrame)
		}
		log.Printf("Track %s has length %d\n", track.Sample.Name, trackLength)
		if trackLength > longestTrackLength {
			longestTrackLength = trackLength
			longestSample = track.Sample
		}
	}

	// log.Printf("Longest Sample: %v\n", longestSample)
	// log.Printf("Highest amplitude point found: %v\n", maxSize)

	// for i, track := range tracks {
	// 	go func(i int, track Track) {
	// 		// longest track doesn't need to be mangles
	// 		if track.Sample == longestSample {
	// 			log.Printf("Not normalizing %s\n", track.Sample.Name)
	// 			return
	// 		}
	// 		log.Printf("%d: Normalizing length of %s to multiple of %d(longest sample was %s)...\n", i, track.Sample.Name, longestTrackLength, longestSample.Name)
	// 		currentLength := 0

	// 		newOutSamples := make([]AudioFrame, len(*(longestSample.OutSamples))+1)
	// 		log.Printf("currentLength: %d allocated %d frames vs %d\n", currentLength, len(*(longestSample.OutSamples)), len(*track.Sample.OutSamples))
	// 		if currentLength < longestTrackLength {
	// 			curFrame := 0
	// 			for currentLength < longestTrackLength {
	// 				log.Printf("Iter for %s\n", track.Sample.Name)
	// 				for _, sampleFrame := range *track.Sample.OutSamples {
	// 					curFrame++
	// 					desiredLen := len(sampleFrame)

	// 					if len(sampleFrame)+currentLength >= longestTrackLength {
	// 						// we don't want to go too far
	// 						desiredLen = longestTrackLength - currentLength
	// 					}

	// 					newOutSamples[curFrame] = sampleFrame[:desiredLen]
	// 					currentLength += desiredLen

	// 					if currentLength >= longestTrackLength {
	// 						break
	// 					}

	// 					// // this frame is incomplete, let's extend it
	// 					// if currentLength < longestTrackLength && desiredLen < FRAME_SIZE {
	// 					// 	log.Printf("Less than! Size: %d vs FRAME_SIZE %d\n", len(newOutSamples[curFrame]), FRAME_SIZE)
	// 					// 	newOutSamples[curFrame] = append(newOutSamples[curFrame], (*track.Sample.OutSamples)[0][:FRAME_SIZE-len(newOutSamples[curFrame])])
	// 					// }
	// 				}
	// 			}

	// 			*track.Sample.OutSamples = newOutSamples
	// 			log.Printf("Normalized %s to %d\n", track.Sample.Name, currentLength)
	// 		}
	// 	}(i, track)
	// }

	// audioPointBuffer contains the calculated mixed audio signal for
	// 1378 upcoming points -- 1/32 second at 44.1 kHz
	// ring.DefaultCapacity = 1378
	ring.DefaultCapacity = 16
	audioFrameBuffer := AudioFrameRingBuffer{}
	audioFrameBuffer.r.SetCapacity(16)
	var mixedAudio []AudioFrame

	mixer := &Mixer{
		MixedAudio: &mixedAudio,
		Tracks:     &tracks,
	}
	f := &Filter{Mode: LOW_PASS, Cutoff: 0.39, Resonance: 0.27}
	screenContext.Filter = f

	//assume 44100 sample rate, mono, 16 bit

	// err = termbox.Init()
	// if err != nil {
	// 	panic(err)
	// }
	// defer termbox.Close()

	// termbox.SetInputMode(termbox.InputEsc | termbox.InputMouse)

	// termbox.Clear(termbox.ColorDefault, termbox.ColorDefault)
	// termbox.Flush()
	// redrawAll(screenContext)

	playbackDone := make(chan bool)
	go func(doneChan chan bool) {
		quit := false
		go func() {
			quit = <-doneChan
		}()

		for {
			if isPlaybackAllowed() {
				if audioFrameBuffer.produced < audioFrameBuffer.Capacity() {
					// runMixer runs the mixer and populates the ring buffer
					runMixer(mixer, &audioFrameBuffer, len(*longestSample.OutSamples), f, screenContext.SelectedSoundset)
				}

				// and we only try to play frames when we have at least 1 waiting
				if audioFrameBuffer.produced >= 1 {
					playAudioFrame(&audioFrameBuffer, &out, stream)
				} else {
					// just play nothingness i guess
					playEmptyAudioFrame(&audioFrameBuffer, &out, stream)
				}
			}

			if quit {
				log.Println("Quitting playback")
				break
			}
		}
	}(playbackDone)

	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt)
	for _ = range c {
		playbackDone <- true
		GPIODoneChan <- true
		break
	}
	// keyboardLoop:
	// 	for {
	// 		switch ev := termbox.PollEvent(); ev.Type {
	// 		case termbox.EventKey:
	// 			if ev.Key == termbox.KeyCtrlC {
	// 				log.Println("CTRL+C pressed")
	// 				log.Println("Sending playback done")
	// 				playbackDone <- true
	// 				GPIODoneChan <- true
	// 				log.Println("Breaking keyboardLoop")
	// 				break keyboardLoop
	// 			}

	// 			if ev.Key == termbox.KeyArrowUp {
	// 				if screenContext.SelectedIndex > 0 {
	// 					screenContext.SelectedIndex--
	// 				}
	// 			}

	// 			if ev.Key == termbox.KeyArrowDown {
	// 				if screenContext.SelectedIndex < len(*screenContext.Tracks)-1 {
	// 					screenContext.SelectedIndex++
	// 				}
	// 			}

	// 			if ev.Ch == 'u' {
	// 				if (*screenContext.Tracks)[screenContext.SelectedIndex].Volume < 100 {
	// 					(*screenContext.Tracks)[screenContext.SelectedIndex].Volume += 10
	// 				}
	// 			}

	// 			if ev.Ch == 'd' {
	// 				if (*screenContext.Tracks)[screenContext.SelectedIndex].Volume > 0 {
	// 					(*screenContext.Tracks)[screenContext.SelectedIndex].Volume -= 10
	// 				}
	// 			}

	// 			if ev.Ch == 'm' {
	// 				(*screenContext.Tracks)[screenContext.SelectedIndex].Volume = 100
	// 			}

	// 			if ev.Ch == 'c' {
	// 				(*screenContext.Tracks)[screenContext.SelectedIndex].Volume = 0
	// 			}

	// 			if ev.Ch == 't' {
	// 				(*screenContext.Filter).Cutoff += 0.01
	// 				if (*screenContext.Filter).Cutoff >= 1.0 {
	// 					(*screenContext.Filter).Cutoff = 0.999999
	// 				}
	// 			}

	// 			if ev.Ch == 'r' {
	// 				(*screenContext.Filter).Cutoff -= 0.01
	// 				if (*screenContext.Filter).Cutoff < 0.0 {
	// 					(*screenContext.Filter).Cutoff = 0.0
	// 				}
	// 			}

	// 			if ev.Ch == 'g' {
	// 				(*screenContext.Filter).Resonance += 0.01
	// 				if (*screenContext.Filter).Resonance >= 1.0 {
	// 					(*screenContext.Filter).Resonance = 0.999999
	// 				}
	// 			}

	// 			if ev.Ch == 'h' {
	// 				(*screenContext.Filter).Resonance -= 0.01
	// 				if (*screenContext.Filter).Resonance < 0.0 {
	// 					(*screenContext.Filter).Resonance = 0.0
	// 				}
	// 			}
	// 		case termbox.EventError:
	// 			panic(ev.Err)
	// 		}
	// 		redrawAll(screenContext)
	// 	}
}

func readChunk(r readerAtSeeker) (id ID, data *io.SectionReader, err error) {
	// 4 bytes
	_, err = r.Read(id[:])
	if err != nil {
		return
	}

	var n int32
	// 4 bytes
	err = binary.Read(r, binary.BigEndian, &n)
	if err != nil {
		return
	}
	off, _ := r.Seek(0, 1)

	// 'n' is length of section
	data = io.NewSectionReader(r, off, int64(n))
	_, err = r.Seek(int64(n), 1)
	return
}

type readerAtSeeker interface {
	io.Reader
	io.ReaderAt
	io.Seeker
}

type ID [4]byte

func (id ID) String() string {
	return string(id[:])
}

type commonChunk struct {
	NumChans      int16
	NumSamples    int32
	BitsPerSample int16
	SampleRate    [10]byte
}

func chk(err error) {
	if err != nil {
		log.Println("Fatal error!")
		log.Println(err)
		panic(err)
	}
}
