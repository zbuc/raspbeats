package main

import (
	"github.com/gordonklaus/portaudio"
	"github.com/nsf/termbox-go"
	"github.com/pelletier/go-toml"
	ring "github.com/zfjagann/golang-ring"

	// "bytes"
	"encoding/binary"
	"fmt"
	"io"
	// "io/ioutil"
	"log"
	"os"
	"sync"
	"time"
)

// The frame size in samples/points.
// 344 is 1/128 of a second at 44.1kHz
// This affects the ticker schedule as well as the latency of input
// processing. I.e. any changes to the mixer are audible on a per-frame
// basis (unless we adpot a point-based ring buffer instead)
// const FRAME_SIZE = 344
// const FRAME_SIZE = 1378
const FRAME_SIZE = 689

const SAMPLE_RATE = 44100

type Looper interface {
	Start()
	Stop()
}

type Loop struct {
}

func GetOutSamples(audio *io.Reader, sampleRate int) []AudioFrame {
	outSamples := make([]AudioFrame, 0)
	for remaining := sampleRate; remaining > 0; remaining -= FRAME_SIZE {
		out := make(AudioFrame, FRAME_SIZE)
		if len(out) > remaining {
			out = out[:remaining]
		}
		err := binary.Read(*audio, binary.BigEndian, out)
		if err == io.EOF {
			log.Printf("io.EOF\n")
			break
		}
		chk(err)
		outSamples = append(outSamples, out)
	}

	return outSamples
}

// Mixes addedFrame * addedFrameVolumePercentage into baseFrame, modifying it in place
func MixFrames(baseFrame *AudioFrame, addedFrame *AudioFrame, addedFrameVolumePercentage float64) {
	// log.Println("MixSamples")
	var longerFrame *AudioFrame
	var shorterFrame *AudioFrame
	if len(*baseFrame) > len(*addedFrame) {
		longerFrame = baseFrame
		shorterFrame = addedFrame
	} else {
		longerFrame = addedFrame
		shorterFrame = baseFrame
	}

	// unfiltered := make([][]int32, len(*longerSample)*4)
	// i := 0
	// for ; i < len(*shorterSample); i++ {
	// 	unfiltered[i] = (*shorterSample)[i]

	// }
	// int* unfiltered = (int *)malloc(lengthOfLongPcmInShorts*4);
	// int i;
	// for(i = 0; i < lengthOfShortPcmInShorts; i++){
	//     unfiltered[i] = shortPcm[i] + longPcm[i];
	// }
	// for(; i < lengthOfLongPcmInShorts; i++){
	//      unfiltered[i] = longPcm[i];
	// }

	// int max = 0;
	// for(int i = 0; i < lengthOfLongPcmInShorts; i++){
	//    int val = unfiltered[i];
	//    if(abs(val) > max)
	//       max = val;
	// }

	// short int *newPcm = (short int *)malloc(lengthOfLongPcmInShorts*2);
	// for(int i = 0; i < lengthOfLongPcmInShorts; i++){
	//    newPcm[i] = (unfilted[i]/max) * MAX_SHRT;
	// }

	// log.Printf("Shorter sample volume: %f, added sample volume: %f", shorterSampleVolume, addedSampleVolumePercentage)
	i := 0
	var p int32
	for i, p = range *longerFrame {
		// Naive algorithm:
		// 		Does not protect against clipping (overflow)
		if len(*shorterFrame) > i {
			if longerFrame == addedFrame {
				(*baseFrame)[i] = int32(addedFrameVolumePercentage*float64(p)) + (*shorterFrame)[i]
			} else {
				(*baseFrame)[i] = p + int32(addedFrameVolumePercentage*float64((*shorterFrame)[i]))
			}
		} else {
			if longerFrame == addedFrame {
				(*baseFrame)[i] = int32(addedFrameVolumePercentage * float64(p))
			} else {
				(*baseFrame)[i] = p
			}
		}
	}

	// unnecessary because of zeroing before mix?
	// // fill any extra space in the frame with emptiness
	// if i < len(*baseFrame) {
	// 	for ; i < len(*baseFrame); i++ {
	// 		(*baseFrame)[i] = 0
	// 	}
	// }

	// log.Printf("Setting mixedSamples: %d\n", len(out))
}

func MixSamples(mixedSamples *[]AudioFrame, addedSample *[]AudioFrame, addedSampleVolumePercentage float64) {
	// log.Println("MixSamples")
	out := make([]AudioFrame, 0)
	var longerSample *[]AudioFrame
	var shorterSample *[]AudioFrame
	shorterSampleVolume := 1.0
	if len(*mixedSamples) > len(*addedSample) {
		longerSample = mixedSamples
		shorterSample = addedSample
		shorterSampleVolume = addedSampleVolumePercentage
	} else {
		longerSample = addedSample
		shorterSample = mixedSamples
	}

	// unfiltered := make([][]int32, len(*longerSample)*4)
	// i := 0
	// for ; i < len(*shorterSample); i++ {
	// 	unfiltered[i] = (*shorterSample)[i]

	// }
	// int* unfiltered = (int *)malloc(lengthOfLongPcmInShorts*4);
	// int i;
	// for(i = 0; i < lengthOfShortPcmInShorts; i++){
	//     unfiltered[i] = shortPcm[i] + longPcm[i];
	// }
	// for(; i < lengthOfLongPcmInShorts; i++){
	//      unfiltered[i] = longPcm[i];
	// }

	// int max = 0;
	// for(int i = 0; i < lengthOfLongPcmInShorts; i++){
	//    int val = unfiltered[i];
	//    if(abs(val) > max)
	//       max = val;
	// }

	// short int *newPcm = (short int *)malloc(lengthOfLongPcmInShorts*2);
	// for(int i = 0; i < lengthOfLongPcmInShorts; i++){
	//    newPcm[i] = (unfilted[i]/max) * MAX_SHRT;
	// }

	// log.Printf("Shorter sample volume: %f, added sample volume: %f", shorterSampleVolume, addedSampleVolumePercentage)
	for x, longerSampleFrame := range *longerSample {
		outFrame := make(AudioFrame, len(longerSampleFrame))
		for i, p := range longerSampleFrame {
			// Naive algorithm:
			// 		Does not protect against clipping (overflow)
			if len(*shorterSample) > x && len((*shorterSample)[x]) > i {
				outFrame[i] = p + int32(shorterSampleVolume*float64((*shorterSample)[x][i]))
			} else {
				outFrame[i] = int32(addedSampleVolumePercentage * float64(p))
			}
		}

		out = append(out, outFrame)
	}

	// log.Printf("Setting mixedSamples: %d\n", len(out))
	*mixedSamples = out
}

func GetAudio(f *os.File) (io.Reader, int) {
	id, data, err := readChunk(f)
	chk(err)
	if id.String() != "FORM" {
		log.Println("bad file format")
		log.Println(id.String())
		return nil, 0
	}
	_, err = data.Read(id[:])
	chk(err)
	if id.String() != "AIFF" {
		log.Println("bad file format")
		log.Println(id.String())
		return nil, 0
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
		case "SSND":
			chunk.Seek(8, 1) //ignore offset and block
			audio = chunk
		default:
			log.Printf("ignoring unknown chunk '%s'\n", id)
		}
	}

	log.Printf("1: %+v\n", audio)
	return audio, int(c.NumSamples)
}

type AudioPoint int32

// "point" is an individual int32 in an audioFrame
func getNextPointChan(audioPointBuffer *AudioPointRingBuffer) chan AudioPoint {
	return make(chan AudioPoint)
}

func getNextFrameChan(audioFrameBuffer *AudioFrameRingBuffer) chan AudioFrame {
	nextFrameChan := make(chan AudioFrame)
	go func() {
		for {
			mixedAudio := audioFrameBuffer.GetFrame()
			if mixedAudio != nil {
				nextFrameChan <- mixedAudio
			}
		}
	}()
	return nextFrameChan
}

func playAudioFrame(audioFrameBuffer *AudioFrameRingBuffer, out *[]int32, stream *portaudio.Stream, nextFrameChan *chan AudioFrame) {
	log.Println("Getting next frame...")
	mixedAudio := <-*nextFrameChan

	log.Println("Got audio frame")
	*out = ([]int32)(mixedAudio)
	chk(stream.Write())
}

type Sample struct {
	SampleRate int
	Audio      *io.Reader
	OutSamples *[]AudioFrame
	Name       string
}

type Track struct {
	Sample *Sample
	Volume int
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
	Tracks        *[]Track
	SelectedIndex int
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

	offset := 2
	for i, track := range *context.Tracks {
		if i == context.SelectedIndex {
			tbprint(0, i+offset, coldef, coldef, fmt.Sprintf("Name: %-50s | Volume: %-10d >", track.Sample.Name, track.Volume))
		} else {
			tbprint(0, i+offset, coldef, coldef, fmt.Sprintf("Name: %-50s | Volume: %-10d", track.Sample.Name, track.Volume))
		}
	}
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

type AudioFrame []int32

// Grab next frame for all samples, perform mixing, add to buffer
func runMixer(mixer *Mixer, audioFrameBuffer *AudioFrameRingBuffer, maxFrameLength int) {
	log.Println("Creating mixed audio frames...")

	mixedFrame := make(AudioFrame, FRAME_SIZE)
	zeroAudio(&mixedFrame)

	// wrap around at the end -- should we just use a ring buffer?
	if mixer.CurrentFrame >= maxFrameLength {
		mixer.CurrentFrame = 0
	}

	log.Println("Step 2: Adding processed tracks together naively")
	// mix every track together
	for _, track := range *mixer.Tracks {
		sample := track.Sample
		// volume := track.Volume
		volumePercentage := float64(track.Volume) / 100.0
		// fmt.Printf("volume: %f\n", volumePercentage)

		// tbLine(fmt.Sprintf("%+v\n", *mixedAudio))
		// tbLine(fmt.Sprintf("%+v\n", *sample.OutSamples))
		// log.Printf("outSamples: %d\n", len(*sample.OutSamples))
		// log.Printf("%v\n", (*sample.OutSamples)[0][1])
		log.Printf("Accessing index %d of %d\n", mixer.CurrentFrame, len(*sample.OutSamples))
		if mixer.CurrentFrame < len(*sample.OutSamples) {
			MixFrames(&mixedFrame, &(*sample.OutSamples)[mixer.CurrentFrame], volumePercentage)
		}
		// log.Printf("mixedAudio: %d\n", len(*mixedAudio))
		// log.Printf("%v\n", (*mixedAudio)[0][1])
	}

	log.Println("Enqueing mixed audio frame...")
	enqueueMixedAudioFrame(audioFrameBuffer, &mixedFrame)

	mixer.CurrentFrame++
	log.Println("Done")
}

func enqueueMixedAudioFrame(audioFrameBuffer *AudioFrameRingBuffer, mixedFrame *AudioFrame) {
	// log.Println("Adding mixed audio frame ptr...")
	// log.Printf("%+v\n", frame)
	audioFrameBuffer.AddFrame(*mixedFrame)
	// log.Println("Added.")
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
	r ring.Ring
	m sync.Mutex
}

func (b *AudioFrameRingBuffer) AddFrame(frame AudioFrame) {
	b.m.Lock()
	defer b.m.Unlock()
	// was passing pointers here but maybe not? don't *want* to realloc...
	b.r.Enqueue(frame)
}

func (b *AudioFrameRingBuffer) GetFrame() AudioFrame {
	b.m.Lock()
	defer b.m.Unlock()
	frame := b.r.Dequeue()
	if frame == nil {
		return nil
	}
	return frame.(AudioFrame)
}

func (b *AudioFrameRingBuffer) Capacity() int {
	b.m.Lock()
	defer b.m.Unlock()

	return b.r.Capacity()
}

type SampleSet map[string][]Sample

func main() {
	loggerFile := SetupLogger()
	defer loggerFile.Close()

	if len(os.Args) != 2 {
		log.Println("missing required argument: soundsets.toml")
		return
	}

	log.Println("Playing.  Press Ctrl-C to stop.")

	tracks := []Track{}
	screenContext := &ScreenContext{Tracks: &tracks}

	confFileName := os.Args[1]
	sampleSetConf, err := toml.LoadFile(confFileName)

	part2 := sampleSetConf.Get("SampleSetConfigs").(*toml.TomlTree)

	for _, sampleSetName := range part2.Keys() {
		log.Println(sampleSetName)
		fileNames := part2.Get(sampleSetName).([]*toml.TomlTree)
		for _, fileNameStruct := range fileNames {
			fileNameMap := fileNameStruct.ToMap()
			fileNamesIfaces := []interface{}(fileNameMap["fileNames"].([]interface{}))

			for _, fileName := range fileNamesIfaces {
				f, err := os.Open(fileName.(string))
				chk(err)
				defer f.Close()
				audio, sampleRate := GetAudio(f)
				outSamples := GetOutSamples(&audio, sampleRate)
				sample := &Sample{
					SampleRate: sampleRate,
					Audio:      &audio,
					OutSamples: &outSamples,
					Name:       fileName.(string),
				}
				track := Track{
					Sample: sample,
					Volume: 0,
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

	fmt.Printf("Longest Sample: %v\n", longestSample)

	for i, track := range tracks {
		fmt.Printf("%d: Normalizing length of %s to multiple of %d...\n", i, track.Sample.Name, longestTrackLength)
		currentLength := 0
		remainingLength := 0
		for _, sampleFrame := range *track.Sample.OutSamples {
			remainingLength += len(sampleFrame)
		}

		fmt.Printf("Remaining Length: %d and currentLength: %d\n", remainingLength, currentLength)
		if remainingLength == 0 && currentLength < longestTrackLength {
			fmt.Println("We need to adjust the length of this sample!")
		}
	}

	// audioPointBuffer contains the calculated mixed audio signal for
	// 1378 upcoming points -- 1/32 second at 44.1 kHz
	// ring.DefaultCapacity = 1378
	ring.DefaultCapacity = 2
	audioFrameBuffer := AudioFrameRingBuffer{}
	var mixedAudio []AudioFrame

	mixer := &Mixer{
		MixedAudio: &mixedAudio,
		Tracks:     &tracks,
	}

	// fmt.Println(mixedAudio)

	//assume 44100 sample rate, mono, 32 bit
	portaudio.Initialize()
	defer portaudio.Terminate()

	out := make([]int32, FRAME_SIZE)
	stream, err := portaudio.OpenDefaultStream(0, 1, SAMPLE_RATE, len(out), &out)
	chk(err)
	defer stream.Close()
	chk(stream.Start())
	defer stream.Stop()

	err = termbox.Init()
	if err != nil {
		panic(err)
	}
	defer termbox.Close()

	termbox.SetInputMode(termbox.InputEsc | termbox.InputMouse)

	termbox.Clear(termbox.ColorDefault, termbox.ColorDefault)
	termbox.Flush()
	redrawAll(screenContext)
	// K_a := termbox.Key{97}

	// ticker time = frame size in samples / 44.1K samples / second
	tickerTime := 1000.0 * (float64(FRAME_SIZE) / SAMPLE_RATE)
	ticker := time.NewTicker(time.Millisecond * time.Duration(tickerTime))
	go func() {
		mixedAudioChan := getNextFrameChan(&audioFrameBuffer)
		for t := range ticker.C {
			log.Println("Tick at", t)
			log.Printf("Buffer: %v\n", audioFrameBuffer.Capacity())
			// runMixer runs the mixer and populates the ring buffer
			runMixer(mixer, &audioFrameBuffer, len(*longestSample.OutSamples))

			// playAudioFrame reads from the ring buffer and writes to the audio
			playAudioFrame(&audioFrameBuffer, &out, stream, &mixedAudioChan)
		}
	}()

keyboardLoop:
	for {
		switch ev := termbox.PollEvent(); ev.Type {
		case termbox.EventKey:
			if ev.Key == termbox.KeyCtrlC {
				ticker.Stop()
				break keyboardLoop
			}

			if ev.Key == termbox.KeyArrowUp {
				if screenContext.SelectedIndex > 0 {
					screenContext.SelectedIndex--
				}
			}

			if ev.Key == termbox.KeyArrowDown {
				if screenContext.SelectedIndex < len(*screenContext.Tracks)-1 {
					screenContext.SelectedIndex++
				}
			}

			if ev.Ch == 'u' {
				if (*screenContext.Tracks)[screenContext.SelectedIndex].Volume < 100 {
					(*screenContext.Tracks)[screenContext.SelectedIndex].Volume += 10
				}
			}

			if ev.Ch == 'd' {
				if (*screenContext.Tracks)[screenContext.SelectedIndex].Volume > 0 {
					(*screenContext.Tracks)[screenContext.SelectedIndex].Volume -= 10
				}
			}

			if ev.Ch == 'm' {
				(*screenContext.Tracks)[screenContext.SelectedIndex].Volume = 100
			}

			if ev.Ch == 'c' {
				(*screenContext.Tracks)[screenContext.SelectedIndex].Volume = 0
			}
		case termbox.EventError:
			panic(ev.Err)
		}
		redrawAll(screenContext)
	}
}

func readChunk(r readerAtSeeker) (id ID, data *io.SectionReader, err error) {
	_, err = r.Read(id[:])
	if err != nil {
		return
	}
	var n int32
	err = binary.Read(r, binary.BigEndian, &n)
	if err != nil {
		return
	}
	off, _ := r.Seek(0, 1)
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
		panic(err)
	}
}
