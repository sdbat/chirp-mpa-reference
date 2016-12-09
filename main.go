package main

import (
	"encoding/binary"
	"fmt"
	"math"
	"time"

	l7g "github.com/immesys/chirp-l7g"
)

var lastNotify time.Time

var mra = make(map[string]*room_anemometer)

func main() {
	lastNotify = time.Now()
	l7g.RunDPA(Initialize, OnNewData, "chirpmicro", "reference_1_1")
}

type room_anemometer struct {
	//map the port number to the matrix index
	port_to_idx [4]int32
	//separation between parts in terms of index in microns
	s_matrix [4][4]float32
	//tofs in microseconds
	tof_matrix [4][4]float32
	//component velocities in m/s
	vel_matrix [4][4]float32
	//scale factors from components to cardinal
	v_scales [3][4][4]float32
	//	vy_scales [4][4]float32
	//	vz_scales [4][4]float32
	//raw cardinal velocities m/s
	vxyz_raw [3]float32
	//stored offset values
	vxyz_offset [3]float32
	//calibrated velocities
	vxyz_cal [3]float32
	//filtered result, output to application
	vxyz_filt [3]float32
	//number of received samples
	num_samples int32
	trace_sum   [4]float32
	trace_filt  [4]float32
	trace_diff  [4]float32
	cal_state   int8
	//anemometer type, 1 = room, 2 = 6" duct
	mode int32
}

func NewDuctAnemometer() *room_anemometer {
	ra := room_anemometer{}
	ra.num_samples = 0
	ra.mode = 2
	//initialize port to index according to geometry
	ra.port_to_idx[0] = 2 //port 0 downstream top
	ra.port_to_idx[1] = 1 //port 1 is upstream bottom
	ra.port_to_idx[2] = 0 //port 2 is upstream top
	ra.port_to_idx[3] = 3 //port 3 is downstream bottom

	baserange := 152400.0 //in microns
	ra.s_matrix[0][1] = baserange
	ra.s_matrix[0][2] = baserange * float32(math.Sqrt(2))
	ra.s_matrix[0][3] = baserange * float32(math.Sqrt(3))
	ra.s_matrix[1][2] = baserange * float32(math.Sqrt(3))
	ra.s_matrix[1][3] = baserange * float32(math.Sqrt(2))
	ra.s_matrix[2][3] = baserange

	for i := 0; i < 4; i++ {
		ra.s_matrix[i][i] = 0.0
		ra.tof_matrix[i][j] = 1.0e-12
	}

	for i := 0; i < 4; i++ {
		for j := i + 1; j < 4; j++ {
			ra.s_matrix[j][i] = ra.s_matrix[i][j]
		}
		for k := 0; k < 4; k++ {
			ra.tof_matrix[i][j] = ra.s_matrix[i][j] / 343.0
		}
	}

	//0 is the top
	ra.v_scales[0][0][2] = float32(math.Cos(45.0 * math.Pi / 180.0))

	//1 is the bottom
	ra.v_scales[1][1][3] = float32(math.Cos(45.0 * math.Pi / 180.0))
	//2 is the diagonal
	ra.v_scales[2][0][3] = float32(math.Cos(45.0*math.Pi/180.0) * math.Cos(30.0*math.Pi/180.0))
	ra.v_scales[2][1][2] = float32(math.Cos(45.0*math.Pi/180.0) * math.Cos(30.0*math.Pi/180.0))

	//flip the matrix across the identity axis
	for i := 0; i < 4; i++ {
		for j := i + 1; j < 4; j++ {
			for k := 0; k < 3; k++ {
				//possibly big bug here
				ra.v_scales[k][i][j] = -ra.v_scales[k][j][i]
			}
		}
	}
}

func NewRoomAnemometer() *room_anemometer {
	ra := room_anemometer{}
	ra.num_samples = 0
	ra.mode = 1
	//initialize port to index according to geometry
	ra.port_to_idx[0] = 1 //port 0 is B in doc
	ra.port_to_idx[1] = 3 //port 1 is D
	ra.port_to_idx[2] = 0 //A
	ra.port_to_idx[3] = 2 //C

	//initialize s_matrix, tof_matrix.
	//Other matrixes are already initialized to 0
	for i := 0; i < 4; i++ {
		for j := 0; j < 4; j++ {
			ra.s_matrix[i][j] = 60000.0
			ra.tof_matrix[i][j] = 174.92
			if i == j {
				ra.s_matrix[i][j] = 0.0
				ra.tof_matrix[i][j] = 1.0e-12 //prevent divide by zero
			}
		}
	}
	ra.v_scales[0][0][2] = float32(math.Cos(30.0 * math.Pi / 180.0))
	ra.v_scales[0][1][2] = float32(math.Cos(30.0 * math.Pi / 180.0))
	ra.v_scales[0][0][3] = float32(math.Cos(54.74*math.Pi/180.0) * math.Sin(60.0*math.Pi/180.0))
	ra.v_scales[0][1][3] = float32(math.Cos(54.74*math.Pi/180.0) * math.Sin(60.0*math.Pi/180.0))
	ra.v_scales[0][2][3] = float32(-math.Cos(54.74 * math.Pi / 180.0))

	ra.v_scales[1][0][1] = 1.0
	ra.v_scales[1][0][2] = float32(math.Sin(30.0 * math.Pi / 180.0))
	ra.v_scales[1][0][3] = float32(math.Cos(54.74*math.Pi/180.0) * math.Cos(60.0*math.Pi/180.0))
	ra.v_scales[1][1][2] = float32(-math.Sin(30.0 * math.Pi / 180.0))
	ra.v_scales[1][1][3] = float32(-math.Cos(54.74*math.Pi/180.0) * math.Cos(60.0*math.Pi/180.0))

	ra.v_scales[2][0][3] = float32(math.Sin(54.74 * math.Pi / 180.0))
	ra.v_scales[2][1][3] = float32(math.Sin(54.74 * math.Pi / 180.0))
	ra.v_scales[2][2][3] = float32(math.Sin(54.74 * math.Pi / 180.0))

	//flip the matrix across the identity axis
	for i := 0; i < 4; i++ {
		for j := i + 1; j < 4; j++ {
			for k := 0; k < 3; k++ {
				//possibly big bug here
				ra.v_scales[k][i][j] = -ra.v_scales[k][j][i]
			}
		}
	}
	return &ra
}

func (ra *room_anemometer) cardinalVelocities() {
	den := [3]float32{0.0, 0.0, 0.0}
	num := [3]float32{0.0, 0.0, 0.0}
	for k := 0; k < 3; k++ {
		for i := 0; i < 4; i++ {
			for j := 0; j < 4; j++ {
				//weighted average, with weights equal to abs value of scale factor
				num[k] = num[k] + ra.vel_matrix[i][j]*ra.v_scales[k][i][j]*float32(math.Abs(float64(ra.v_scales[k][i][j])))
				den[k] = den[k] + float32(math.Abs(float64(ra.v_scales[k][i][j])))
			}
		}
		ra.vxyz_raw[k] = num[k] / den[k]
	}
}

func (ra *room_anemometer) filterTrace(coeff float32, i uint8) {
	if ra.num_samples <= 20 {
		ra.trace_filt[i] = ra.trace_sum[i]
	} else {
		ra.trace_filt[i] = ra.trace_filt[i]*coeff + ra.trace_sum[i]*(1-coeff)
	}
	ra.trace_diff[i] = ra.trace_sum[i] - ra.trace_filt[i]
}

func (ra *room_anemometer) filterVelocity(coeff float32) {
	for k := 0; k < 3; k++ {
		if ra.num_samples <= 20 {
			ra.vxyz_filt[k] = ra.vxyz_raw[k]
		} else {
			ra.vxyz_filt[k] = ra.vxyz_filt[k]*coeff + ra.vxyz_raw[k]*(1-coeff)
		}

	}
}

func (ra *room_anemometer) calibrateVelocity(samps int32) {

	for k := 0; k < 3; k++ {
		if ra.num_samples == samps {
			ra.vxyz_offset[k] = ra.vxyz_filt[k]
		}
		ra.vxyz_cal[k] = ra.vxyz_filt[k] - ra.vxyz_offset[k]
	}

}

func Initialize(emit l7g.Emitter) {
	//We actually do not do any initialization in this implementation, but if
	//you want to, you can do it here.
}

// OnNewData encapsulates the algorithm. You can store the emitter and
// use it asynchronously if required. You can see the documentation for the
// parameters at https://godoc.org/github.com/immesys/chirp-l7g
func OnNewData(popHdr *l7g.L7GHeader, h *l7g.ChirpHeader, emit l7g.Emitter) {
	// Define some magic constants for the algorithm
	magic_count_tx := -3.125

	//fmt.Printf("Device id: %s\n", popHdr.Srcmac)
	ra, ok := mra[popHdr.Srcmac]
	if ok == false {
		fmt.Printf("No key for: %s, creating new RA\n", popHdr.Srcmac)
		if 0 {

		} else {
			mra[popHdr.Srcmac] = NewRoomAnemometer()
		}

		ra = mra[popHdr.Srcmac]
		fmt.Println(ra)
	}

	// Create our output data set. For this reference implementation,
	// we emit one TOF measurement for every raw TOF sample (no averaging)
	// so the timestamp is simply the raw timestamp obtained from the
	// Border Router. We also identify the sensor simply from the mac address
	// (this is fine for most cases)
	odata := l7g.OutputData{
		Timestamp: popHdr.Brtime,
		Sensor:    popHdr.Srcmac,
	}
	toprint := false
	isprimary := false
	// For each of the four measurements in the data set
	for set := 0; set < 4; set++ {
		isprimary = false
		// For now, ignore the data read from the ASIC in TXRX
		if int(h.Primary) == set {
			isprimary = true
		}

		// alias the data for readability. This is the 70 byte dataset
		// read from the ASIC
		data := h.Data[set]

		//The first six bytes of the data
		tof_sf := binary.LittleEndian.Uint16(data[0:2])
		tof_est := binary.LittleEndian.Uint16(data[2:4])
		intensity := binary.LittleEndian.Uint16(data[4:6])

		//Load the complex numbers
		iz := make([]int16, 16)
		qz := make([]int16, 16)
		for i := 0; i < 16; i++ {
			qz[i] = int16(binary.LittleEndian.Uint16(data[6+4*i:]))
			iz[i] = int16(binary.LittleEndian.Uint16(data[6+4*i+2:]))
		}

		//Find the largest complex magnitude (as a square). We do this like this
		//because it more closely mirror how it would be done on an embedded device
		// (actually because I copied the known-good firestorm implementation)
		magsqr := make([]uint64, 16)
		magmax := uint64(0)
		for i := 0; i < 16; i++ {
			magsqr[i] = uint64(int64(qz[i])*int64(qz[i]) + int64(iz[i])*int64(iz[i]))
			if magsqr[i] > magmax {
				magmax = magsqr[i]
			}
		}
		txi := ra.port_to_idx[h.Primary]
		rxi := ra.port_to_idx[set]
		//		fmt.Printf("Tx: %d Rx: %d Tdx: %d Rdx: %d\n", h.Primary, set, txi, rxi)
		if isprimary == false {
			//Find the first index to be greater than half the max (quarter the square)
			quarter := magmax / 4
			less_idx := 0
			greater_idx := 0
			for i := 0; i < 16; i++ {
				if magsqr[i] < quarter {
					less_idx = i
				}
				if magsqr[i] > quarter {
					greater_idx = i
					break
				}
			}

			//Convert the squares into normal floating point
			less_val := math.Sqrt(float64(magsqr[less_idx]))
			greater_val := math.Sqrt(float64(magsqr[greater_idx]))
			half_val := math.Sqrt(float64(quarter))
			//CalPulse is in microseconds
			freq := float64(tof_sf) / 2048 * float64(h.CalRes[set]) / (float64(h.CalPulse) / 1000)
			//Linearly interpolate the index (the index is related to time of flight because it is regularly sampled)
			lerp_idx := float64(less_idx) + (half_val-less_val)/(greater_val-less_val)
			//Fudge the result with magic_count_tx and turn into time of flight
			tof := (lerp_idx + float64(magic_count_tx)) / freq * 8
			_ = tof_est
			_ = intensity
			//		fmt.Printf("SEQ %d ASIC %d primary=%d\n", h.Seqno, set, h.Primary)
			//		fmt.Printf("tof: %.2f us\n", tof*1000000)
			//		fmt.Println("freq: ", freq)
			if toprint {

				//We print these just for fun / debugging, but this is not actually emitting the data
				fmt.Printf("SEQ %d ASIC %d primary=%d\n", h.Seqno, set, h.Primary)
				fmt.Println("lerp_idx: ", lerp_idx)
				fmt.Println("tof_sf: ", tof_sf)
				fmt.Println("freq: ", freq)
				fmt.Printf("tof: %.2f us\n", tof*1000000)
				fmt.Println("intensity: ", intensity)
				fmt.Println("tof chip estimate: ", tof_est)
				fmt.Println("tof 50us estimate: ", lerp_idx*50)
				fmt.Println("data: ")
				for i := 0; i < 16; i++ {
					fmt.Printf(" [%2d] %6d + %6di (%.2f)\n", i, qz[i], iz[i], math.Sqrt(float64(magsqr[i])))
				}
				fmt.Println(".")
			}

			ra.tof_matrix[txi][rxi] = float32(tof * 1000000.0)
			ra.vel_matrix[txi][rxi] = 0.5 * (ra.s_matrix[txi][rxi]/ra.tof_matrix[txi][rxi] -
				ra.s_matrix[rxi][txi]/ra.tof_matrix[rxi][txi])

			//Append this time of flight to the output data set
			//For more "real" implementations, this would likely
			//be a rolling-window smoothed time of flight. You do not have
			//to base this value on just the data from this set and
			//you do not have to emit every time either (downsampling is ok)
			odata.Tofs = append(odata.Tofs, l7g.TOFMeasure{
				Src: int(h.Primary),
				Dst: set,
				Val: tof * 1000000})
		} else {
			//isprimary == true
			sum := float32(0)
			for i := 0; i < 16; i++ {
				sum = sum + float32(magsqr[i])
			}
			ra.trace_sum[set] = sum / float32(16.0*32768.0*32768.0)
		}

	} //end for each of the four measurements
	ra.num_samples = ra.num_samples + 1
	ra.cardinalVelocities()

	ra.filterVelocity(0.99)
	ra.filterTrace(0.99, h.Primary)
	ra.calibrateVelocity(500)

	if popHdr.Srcmac == "b0e7769c5e1c465a" {
		fmt.Printf("%d, %.3f, %.3f\n", ra.num_samples, ra.vxyz_cal, ra.tof_matrix)
	}

	// Now we would also emit the velocities. I imagine this would use
	// the averaged/corrected time of flights that are emitted above
	// (when they are actually averaged/corrected)
	// For now, just a placeholder
	odata.Velocities = append(odata.Velocities, l7g.VelocityMeasure{X: float64(ra.vxyz_cal[0]), Y: float64(ra.vxyz_cal[1]), Z: float64(ra.vxyz_cal[2])})

	// You can also add some extra data here, maybe intermittently like
	if time.Now().Sub(lastNotify) > 5*time.Second {
		odata.Extradata = append(odata.Extradata, fmt.Sprintf("anemometer %s build is %d", popHdr.Srcmac, h.Build))
		lastNotify = time.Now()
	}

	//Emit the data on the SASC bus
	emit.Data(odata)
}
