#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <signal.h>
#include <pigpio.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <sys/stat.h>
/* 
 * TRGALNIK v0.4
 * PIGPIO implementation
 * pin numbers are GPIO numbers.
 */
#define PinMotor 12 //GPIO Pin for Motor PWM
#define PinMotorDir 27 //GPIO Pin for Motor direction
#define MotorFullRotationSteps 600 //number of steps per rotation (motor)
#define PinEncA 23 //GPIO Pin for Encoder channel A
#define PinEncB 24 //GPIO Pin for Encoder channel B
#define PinEncZ 25 //GPIO Pin for Encoder index channel Z
#define BaartCs 0 //Baart (ADC) SPI channel
#define LCRangeN 2000 //Load cell max range in Newtons
#define LCVoltage 10 //Load cell max voltage (at LCRangeN)
#define LCChannel 2 //Load cell ADC channel [1-8]
#define AUXChannel 4 //AUX ADC channel [1-8]
#define AUXVoltage 1.02 //AUX max voltage
#define dilej 101 //ADC sampling sleep duration in us. Values less then 101 are implemented using a busy loop and thus not recommended.
#define fmaxpeaks 3 //Cumulative number of maximum force peaks to stop the motor.
#define fnmotor ".motor.txt"  //suffix for motor data file name
#define fnenc ".encoder.txt" //suffix for encoder data file name
#define fnlc ".loadcell.txt" //suffix for adc data file name
#define fneq ".eq.txt" //suffix for time equalized data file name

using namespace std;

volatile bool keepRunning = true;
volatile uint32_t startTick;
pthread_t motorthread;

class Frequency{
	public:
	typedef struct
	{
	   uint32_t first_tick;
	   uint32_t last_tick;
	   uint32_t pulse_count;
	} freqData;
	freqData oldFreqData, currentFreqData;
	volatile int g_reset_counts;
};

Frequency _frequency;
void FrequencyCallback(int gpio,int level, uint32_t tick){
	if (gpio!=PinEncA) return;
	_frequency.currentFreqData.last_tick = tick;
	if (level == 1) _frequency.currentFreqData.pulse_count++;
	if (_frequency.g_reset_counts)
	{
		_frequency.g_reset_counts = 0;
		_frequency.currentFreqData.first_tick = tick;
		_frequency.currentFreqData.last_tick = tick;
		_frequency.currentFreqData.pulse_count = 0;
	}
};

class Report{
	public:
	enum eDataType { motor, encoder, loadcell};
	uint32_t tick;
	string value;
	eDataType type;
	Report(uint32_t t,string v,eDataType edt){
		tick=t;
		value=v;
		type=edt;
	};
	
};
vector<Report> vMotor,vEnc,vLC;
//#time #motor #enc #lc
vector<Report> ReportOrdered(void){
	vector<Report> result;
	vector<Report> *largest=&vEnc,*youngest;
	vector<Report>::iterator iE=vEnc.begin(), iM=vMotor.begin(), iLC=vLC.begin();
	if (vLC.size()>(*largest).size()) largest=&vLC;
	while((*largest).size()>0){
		youngest=&vEnc;
		if ((*iLC).tick<(*iE).tick) youngest=&vLC;
		if ((*iM).tick<(*youngest).front().tick) youngest=&vMotor;
	result.push_back((*youngest)[0]);
	youngest->erase(youngest->begin());
	largest=&vMotor;
	if (vLC.size()>(*largest).size()) largest=&vLC;
		if (vEnc.size()>(*largest).size()) largest=&vEnc;
	}
	return result;
}

inline bool fileExists(string name){
	struct stat buffer;
	return (stat (name.c_str(),&buffer)==0);
}

int CheckFiles(string fname){
	int ex=0;
	if (fileExists(fname+fnmotor)==1) {
		cout<<"File: "<<fname+fnmotor<<" exists!\n";
		ex++;
	}
	if (fileExists(fname+fnenc)==1) {
		cout<<"File: "<<fname+fnenc<<" exists!\n";
		ex++;
	}
	if (fileExists(fname+fnlc)==1) {
		cout<<"File: "<<fname+fnlc<<" exists!\n";
		ex++;
	}
	return ex;
}

void ReportSave(vector<Report> *data, string fname){
	FILE *file;
	cout<<"file name:"<<fname<<"\n";
	file=fopen(fname.c_str(),"w");
	for (int i=0; i<(*data).size(); i++){
		fprintf(file,"%s\t%s\n",to_string((*data)[i].tick-startTick).c_str(),(*data)[i].value.c_str());
	}
	fclose(file);
}

void ReportsSave(string fname){
	ReportSave(&vMotor,fname+fnmotor);
	ReportSave(&vEnc,fname+fnenc);
	ReportSave(&vLC,fname+fnlc);
}

void intHandler(int sig_num){
	signal(SIGINT, intHandler);
	keepRunning = false;
	int freqSet=gpioGetPWMfrequency(PinMotor);
	gpioHardwarePWM(PinMotor,freqSet,0);
	vMotor.push_back(Report(gpioTick(),"OFF:break",Report::motor));
	pthread_cancel(motorthread);
	printf("\e[?25h\n");
}

class Encoder{
	int gpioA, gpioB, levA, levB, lastGpio,lastEncoded;
	void _pulse4(int gpio, int level, uint32_t tick){
	    if (gpio == gpioA) levA = level; else levB = level;
	    int MSB = levA;
	    int LSB = levB;
	    int encoded = (MSB << 1) | LSB;
	    int sum = (lastEncoded << 2) | encoded;
	    if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) position--;
	    if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) position++;
	    lastEncoded = encoded;
	}
	/* Need a static callback to link with C. */
	static void _pulse4Ex(int gpio, int level, uint32_t tick, void *user){
	Encoder *enc = (Encoder *) user;
	enc->_pulse4(gpio, level, tick); /* Call the instance callback. */
	}
	void _pulse(int gpio, int level, uint32_t tick){
		if (gpio == gpioA) levA = level; 
		else levB = level;
		if (gpio != lastGpio) // debounce 
		{
			lastGpio = gpio;
			if ((gpio == gpioA) && (level == 1)){
				if (levB) position--;
			}
			else if ((gpio == gpioB) && (level == 1)){
				if (levA) position++;
			}
		}
	}
	/* Need a static callback to link with C. */
	static void _pulseEx(int gpio, int level, uint32_t tick, void *user){
		Encoder *enc = (Encoder *) user;
		enc->_pulse(gpio, level, tick); /* Call the instance callback. */
	}
	public:
	int32_t position;
	/*
	This function establishes a rotary encoder on gpioA and gpioB.
	When the encoder is turned the callback function is called.
	*/
	Encoder(int _gpioA, int _gpioB){
		position=0;
		gpioA = _gpioA;
		gpioB = _gpioB;
		levA=0;
		levB=0;
		lastGpio = -1;
		gpioSetMode(gpioA, PI_INPUT);
		gpioSetMode(gpioB, PI_INPUT);
		/* pull up is needed as encoder common is grounded */
		gpioSetPullUpDown(gpioA, PI_PUD_UP);
		gpioSetPullUpDown(gpioB, PI_PUD_UP);
		/* monitor encoder level changes */
		gpioSetAlertFuncEx(gpioA, _pulse4Ex, this);
		gpioSetAlertFuncEx(gpioB, _pulse4Ex, this);
	}
	/*
	This function releases the resources used by the decoder.
	*/
	void Encoder_cancel(void){
		gpioSetAlertFuncEx(gpioA, 0, this);
		gpioSetAlertFuncEx(gpioB, 0, this);
	}
};
Encoder *_encoder;

class motorData{
	public: 
	int d; // d .. distance [mm] <>0; if negative, the direction is reversed.
	double v; // v .. speed [mm/s] >0;
	motorData(double _v, int _d){
		v=_v;
		d=_d;
	}
};

class cycleData: public motorData{
	public:
	int n; // n .. number of cycles
	cycleData(double _v, int _d, int _n): motorData(_v,_d) {
		n=_n;
	}
};

// Motor control using software generated pulses.
// Avoid usage unless precise position is required, as frequency and duty cycle accuracy is not good.
// d .. distance [mm] <>0; if negative, the direction is reversed.
// v .. speed [mm/s] >0;
// motorData mdata 
void *motorPulse (void *mdata){
	if (!keepRunning) pthread_exit(NULL);
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	motorData data=*(class motorData *)mdata;
	if (data.d>0) gpioWrite (PinMotorDir, 1);
	else gpioWrite (PinMotorDir, 0);
	double freqReq = data.v * MotorFullRotationSteps; //zeljena frekvenca v Hz
	int n = MotorFullRotationSteps * abs(data.d); //st. pulzov
	int pause = 1000000.0/data.v/MotorFullRotationSteps/2/1.02;
	int time = abs(1000000.0*data.d/freqReq*MotorFullRotationSteps);
	cout<<"data.d:"<<data.d<<"mm data.v:"<<data.v<<"mm/s freqReq:"<<freqReq<<"Hz rotstep:"<<MotorFullRotationSteps<<"/mm pause:"<<pause/1000000.0<<"s time:"<<time/1000000.0<<"\n";
	vMotor.push_back(Report(gpioTick(),"ON:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
        for (int i=0;i<n;i++){
	pthread_testcancel();
	    gpioWrite(PinMotor,1);
	    gpioDelayBusy(pause);
	    gpioWrite(PinMotor,0);
	    gpioDelayBusy(pause);
            if (!keepRunning) break;
        }
	vMotor.push_back(Report(gpioTick(),"OFF:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
	pthread_exit(NULL);
};

// Motor control using hardware PWM.
// Very high frequency and duty cycle stability. Can overshoot for a couple of pulses (<0.01% FS).
// d .. distance [mm] <>0; if negative, the direction is reversed.
// v .. speed [mm/s] >0;
// motorData mdata 
void *motorPWM (void *mdata){
	if (!keepRunning) pthread_exit(NULL);
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	motorData data=*(class motorData *)mdata;
	if (data.d>0) gpioWrite (PinMotorDir, 1);
	else gpioWrite (PinMotorDir, 0);
	double freqReq = data.v * MotorFullRotationSteps; //zeljena frekvenca v Hz
	gpioHardwarePWM(PinMotor,freqReq,0);
	int freqSet=gpioGetPWMfrequency(PinMotor);
	int pause = abs(1000000.0*data.d/freqSet*MotorFullRotationSteps);
	cout<<"data.d:"<<data.d<<"mm data.v:"<<data.v<<"mm/s freqReq:"<<freqReq<<"Hz freqSet:"<<freqSet<<"Hz rotstep:"<<MotorFullRotationSteps<<"/mm pause:"<<pause/1000000.0<<"s\n";
	pthread_testcancel();
	gpioHardwarePWM(PinMotor,freqReq,500000);
	vMotor.push_back(Report(gpioTick(),"ON:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
	gpioDelay(pause);
	gpioHardwarePWM(PinMotor,freqReq,0);
	vMotor.push_back(Report(gpioTick(),"OFF:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
	pthread_exit(NULL);
};

// Motor cycling using hardware PWM
// d .. distance [mm] <>0; if negative, the direction is reversed.
// v .. speed [mm/s] >0;
// c .. number of cycles
// cycledata cdata
void *motorPWMcycle (void *cdata){
	if (!keepRunning) pthread_exit(NULL);
	pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
	cycleData data=*(class cycleData *)cdata;
	if (data.d>0) gpioWrite (PinMotorDir, 1);
	else gpioWrite (PinMotorDir, 0);
	double freqReq = data.v * MotorFullRotationSteps; //zeljena frekvenca v Hz
	gpioHardwarePWM(PinMotor,freqReq,0);
	int freqSet=gpioGetPWMfrequency(PinMotor);
	int pause = abs(1000000.0*data.d/freqSet*MotorFullRotationSteps);
	pthread_testcancel();
	for (int n=0;n<data.n;n++){
	//CYCLE UP/DOWN START
            cout<<"count:"<<n<<" data.d:"<<data.d<<"mm data.v:"<<data.v<<"mm/s freqReq:"<<freqReq<<"Hz freqSet:"<<freqSet<<"Hz rotstep:"<<MotorFullRotationSteps<<"/mm pause:"<<pause/1000000.0<<"s\n";
            gpioHardwarePWM(PinMotor,freqReq,500000);
	    vMotor.push_back(Report(gpioTick(),"ON:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
	    gpioDelay(pause);
	    //CYCLE DOWN/UP START
            cout<<"count:"<<n<<" data.d:"<<-data.d<<"mm data.v:"<<data.v<<"mm/s freqReq:"<<freqReq<<"Hz freqSet:"<<freqSet<<"Hz rotstep:"<<MotorFullRotationSteps<<"/mm pause:"<<pause/1000000.0<<"s\n";
	    if (data.d>0) gpioWrite (PinMotorDir, 0);
	    else gpioWrite (PinMotorDir, 1);
	    vMotor.push_back(Report(gpioTick(),"ON:"+to_string(data.v)+";"+to_string(-data.d)+";"+to_string(pause),Report::motor));
	    gpioDelay(pause);
    }
	gpioHardwarePWM(PinMotor,freqReq,0);
	vMotor.push_back(Report(gpioTick(),"OFF:"+to_string(data.v)+";"+to_string(data.d)+";"+to_string(pause),Report::motor));
	pthread_exit(NULL);
};

void *readADC(void *){
	int spi=spiOpen(0,500000,0),val,valtr;
	double V,f,Vtr;
	char tx[3],rx[3],txt[3],rxt[3]; //tx&rx LC; txt&rxt TR
	tx[0]=4+(LCChannel>>2);
	tx[1]=(LCChannel&3)<<6;
	tx[3]=0;
	txt[0]=4+(AUXChannel>>2);
	txt[1]=(AUXChannel&3)<<6;
	txt[3]=0;
	while(keepRunning){
		spiXfer(spi,tx,rx,3); //send the three bytes to the A/D in the format the A/D's datasheet explains
		spiXfer(spi,txt,rxt,3);
		val=((rx[1] & 15 ) << 8) | (rx[2] & 255); //use AND operation with the second byte to get the last  4 bits, and then make way for the third data byte with the "move 8 bits to left" << 8 operation
		valtr=((rxt[1] & 15 ) << 8) | (rxt[2] & 255);
		V=val/4096.0*LCVoltage;
		f=V*LCRangeN/LCVoltage;
		Vtr=valtr/4096.0*AUXVoltage;
		vLC.push_back(Report(gpioTick(),to_string(V)+"\t"+to_string(f)+"\t"+to_string(Vtr),Report::loadcell));
		gpioDelay(dilej);
	}
	spiClose(spi);
	pthread_exit(NULL);
}

void *printADC(void *){
	while(keepRunning){
		if (vLC.size()>0){
			printf("%s\t%s           \n",to_string(vLC[vLC.size()-1].tick-startTick).c_str(),vLC[vLC.size()-1].value.c_str());
			fflush(stdout);
		}
		gpioDelay(500000);
	}
	pthread_exit(NULL);
}

void *readADCstop(void * _fmax){
	int spi=spiOpen(0,500000,0);
	char tx[3],rx[3],txt[3],rxt[3]; //tx&rx LC; txt&rxt TR
	tx[0]=4+(LCChannel>>2);
	tx[1]=(LCChannel&3)<<6;
	tx[3]=0;
	txt[0]=4+(AUXChannel>>2);
	txt[1]=(AUXChannel&3)<<6;
	txt[3]=0;
	int fmax=*(int*)_fmax, fmaxcounter=0;
	double V, f,Vtr;//, oldV;
	while(keepRunning){
		spiXfer(spi,tx,rx,3);
		V=((rx[1] & 15 ) << 8) | (rx[2] & 255);
		V=V/4096.0*LCVoltage;
		f=V*LCRangeN/LCVoltage;
		spiXfer(spi,txt,rxt,3);
		Vtr=((rxt[1] & 15 ) << 8) | (rxt[2] & 255);
		Vtr=Vtr/4096.0*AUXVoltage;
		vLC.push_back(Report(gpioTick(),to_string(V)+"\t"+to_string(f)+"\t"+to_string(Vtr),Report::loadcell));
		cout<<" V:"<<V<<" N:"<<f<<" T:"<<Vtr<<"                                                \r"; 
		fflush(stdout);
		if (f>fmax) fmaxcounter++;
		if (fmaxcounter>=fmaxpeaks){
			gpioHardwarePWM(PinMotor,0,0);
			vMotor.push_back(Report(gpioTick(),"OFF:maxF_reached",Report::motor));
			keepRunning=false;
//			vLC.push_back(Report(gpioTick(),"max F:"+to_string(fmax),Report::loadcell)); //this was present in old files
			pthread_cancel(motorthread);
		}
		gpioDelay(dilej);
	}
	spiClose(spi);
	pthread_exit(NULL);
}

void *readEnc(void *){
	while (keepRunning){
		vEnc.push_back(Report(gpioTick(),to_string(_encoder->position),Report::encoder));
		gpioDelay(dilej);
		cout<<"\rt:"<<gpioTick()-startTick<<" enc:"<<_encoder->position<<"          ";
		fflush(stdout);
	}
	pthread_exit(NULL);
}

void initialize(void){
	cout<<"Initialising...\n";
//	gpioCfgBufferSize(1000); 1000ms buffer (106MB) if needed for superfast speeds or <um encoder
	gpioCfgClock(1,1,0); //1us sampling rate)
	if (gpioInitialise() < 0) {
		fprintf(stderr, "Unable to setup pigpio:%s\n",strerror(errno));
		exit(2);
	}
	signal(SIGINT, intHandler);
	gpioSetMode(PinMotor, PI_ALT0); //ALT0=PWM
	gpioSetMode(PinMotorDir, PI_OUTPUT);
	gpioSetMode(PinEncA, PI_INPUT);
	gpioSetMode(PinEncB, PI_INPUT);
	gpioSetMode(PinEncZ, PI_INPUT);
	_encoder = new Encoder(PinEncA, PinEncB);
	startTick=gpioTick();
//	cout<<"MaxMicros:"<<gpioWaveGetMaxMicros()/1e6<<"s MaxPulses:"<<gpioWaveGetMaxPulses()<<" MaxCBS:"<<gpioWaveGetMaxCbs()<<"\n";
}

int cleanup(void){
	delete _encoder;
	gpioSetMode(PinMotorDir, PI_INPUT);
	gpioSetMode(PinMotor, PI_INPUT);
	return 0;
}

void protocolLoadcellTest(void){
	pthread_t tl,tlp;
	void *sl,*slp;
	pthread_create(&tl, NULL, &readADC, (void *) NULL);
	pthread_create(&tlp, NULL, &printADC, (void *) NULL);
	pthread_join(tl,&sl);
	pthread_join(tlp,&slp);
}

// Protocol MOVE
// d .. distance [mm] <>0; if negative, the direction is reversed.
// v .. speed [mm/s] >0;
void protocolPremik(double v, int d){
	motorData data (v,d);
	pthread_t te,tl;
	void *sm,*se,*sl;
	pthread_create(&motorthread, NULL, &motorPWM,(void *)&data);
//	pthread_create(&motorthread, NULL, &motorPulse,(void *)&data);
	pthread_create(&te, NULL, &readEnc,(void *) NULL);
	pthread_create(&tl, NULL, &readADC,(void *) NULL);
	pthread_join(motorthread,&sm);
	keepRunning=false;
	pthread_join(te,&se);
	pthread_join(tl,&sl);
}

// Protocol Preconditioning
// hz .. Frequency [Hz]
// d .. Amplitude [mm]
// c .. Number of cycles
void protocolPreconditioning (double hz, int d, int c){
	cycleData cdata(2*hz*abs(d),d,c);
	pthread_t te,tl;
	void *sm,*se,*sl;
	pthread_create(&tl, NULL, &readADC, (void *) NULL);
	pthread_create(&te, NULL, &readEnc, (void *) NULL);
        pthread_create(&motorthread, NULL, &motorPWMcycle,(void *) &cdata);
        pthread_join(motorthread,&sm);
	keepRunning=false;
	pthread_join(te,&se);
	pthread_join(tl,&sl);
}

// Protocol TEST/Preload
// v .. speed [mm/s] >0;
// d .. max distance [mm] <>0; if negative, the direction is reversed.
// fmax .. max force [Nm]
void protocolTest(double v, int d, int fmax){
	pthread_t te,tl;
	void *sm,*se,*sl;
	motorData mdata (v,d);
	pthread_create(&tl, NULL, &readADCstop, (void *) &fmax);
	pthread_create(&te, NULL, &readEnc, (void *) NULL);
	pthread_create(&motorthread, NULL, &motorPWM,(void *) &mdata);
	pthread_join(motorthread,&sm);
	keepRunning=false;
	pthread_join(te,&se);
	pthread_join(tl,&sl);
}

// Protocol Equalize
// Convert to equally spaced time series data
// step .. time step [us]
void protocolEqualize(int step,string fname){
	if (fileExists(fname+fneq)){
	    cout<<"File: "<<fname+fneq<<" exists! Nothing to do.\n";
	    exit(3);
	};
	FILE *file;
	cout<<"Creating "<<fname+fneq<<"...\n";
	file=fopen(((string)fname+fneq).c_str(),"w");
	fprintf(file,"%s\n","#time\t#encoder [um]\t#loadcell [N]\t#AUX [V]\t#motor");
	ifstream im(fname+fnmotor),ie(fname+fnenc),il(fname+fnlc);
	string vm,vmp;
	uint32_t tm=0,tmp=0,te=0,tep=0,tl=0,tlp=0,i=0;
	int32_t ve,vep;
	double vlv=0,vlvp=0,vlf=0,vlfp=0,vlt=0,vltp=0;
	while (true)
	{
		while (tm<=i && im.good()){tmp=tm; vmp=vm; im>>tm>>vm;}// cout<<"i:"<<i<<"tm:"<<tm<<"tmp:"<<tmp<<"vm:"<<vm<<"vmp:"<<vmp<<"good:"<<im.good()<<"eof:"<<im.eof()<<"fail:"<<im.fail()<<"bad:"<<im.bad()<<"rdstate:"<<im.rdstate()<<"\n";}
		if (tm<=i && !im.good()) vmp=vm;
		while (te<=i && ie.good()) {tep=te; vep=ve; ie>>te>>ve;}
		if (te<=i && !ie.good()) vep=ve;
		while (tl<=i && il.good()) {tlp=tl; vlvp=vlv; vlfp=vlf; vltp=vlt; il>>tl>>vlv>>vlf>>vlt;}
		if (tl<=i && !il.good()) {vlvp=vlv; vlfp=vlf; vltp=vlt;}
		fprintf(file,"%u\t%i\t%f\t%f\t%s\n",i,vep,vlfp,vltp,vmp.c_str());
		i+=step;
		if( (im>>ws).eof() && (ie>>ws).eof() && (il>>ws).eof()){
			fprintf(file,"%u\t%i\t%f\t%f\t%s\n",i,ve,vlf,vlt,vm.c_str());
			break;
		}
	}
	fclose(file);
	im.close();
	ie.close();
	il.close();
}

// Protocol Equalize
// Convert to equally spaced time series data
// for older files without AUX channel data.
void protocolEqualizeNotrigger(int step,string fname){
	if (fileExists(fname+fneq)){
	    cout<<"File: "<<fname+fneq<<" exists! Nothing to do.\n";
	    exit(3);
	};
	FILE *file;
	cout<<"Creating "<<fname+fneq<<"...\n";
	file=fopen(((string)fname+fneq).c_str(),"w");
	fprintf(file,"%s\n","#time\t#encoder [um]\t#loadcell [N]\t#trigger [V]\t#motor");
	ifstream im(fname+fnmotor),ie(fname+fnenc),il(fname+fnlc);
	string vm,vmp;
	uint32_t tm=0,tmp=0,te=0,tep=0,tl=0,tlp=0,i=0;
	int32_t ve,vep;
	double vlv=0,vlvp=0,vlf=0,vlfp=0,vlt=0,vltp=0;
	while (true)
	{
		while (tm<=i && im.good()){tmp=tm; vmp=vm; im>>tm>>vm;}// cout<<"i:"<<i<<"tm:"<<tm<<"tmp:"<<tmp<<"vm:"<<vm<<"vmp:"<<vmp<<"good:"<<im.good()<<"eof:"<<im.eof()<<"fail:"<<im.fail()<<"bad:"<<im.bad()<<"rdstate:"<<im.rdstate()<<"\n";}
		if (tm<=i && !im.good()) vmp=vm;
		while (te<=i && ie.good()) {tep=te; vep=ve; ie>>te>>ve;}
		if (te<=i && !ie.good()) vep=ve;
		while (tl<=i && il.good()) {tlp=tl; vlvp=vlv; vlfp=vlf; il>>tl>>vlv>>vlf;}
		if (tl<=i && !il.good()) {vlvp=vlv; vlfp=vlf;}
		fprintf(file,"%u\t%i\t%f\t%s\n",i,vep,vlfp,vmp.c_str());
		i+=step;
		if( (im>>ws).eof() && (ie>>ws).eof() && (il>>ws).eof()){
			fprintf(file,"%u\t%i\t%f\t%s\n",i,ve,vlf,vm.c_str());
			break;
		}
	}
	fclose(file);
	im.close();
	ie.close();
	il.close();
}

void printarg(char arg){
	switch (arg){
	case 'D': 
		printf("\tD\tdoza test \n");
		break;
	case 'P': 
		printf("\tP\t\e[1mPremik\e[0m hitrost[mm/s] razdalja[mm]\n");
		break;
	case 'C':
		printf("\tC\t\e[1mPreconditioning\e[0m frekvenca[Hz] amplituda[mm] stevilo-ciklov[/]\n");
		break;
	case 'T':
		printf("\tT\t\e[1mTest\e[0m hitrost[mm/s] maxrazdalja[mm] maxsila[N]\n");
		break;
	case 'E':
		printf("\tE\t\e[1mEqualize\e[0m time_step[us] name\n");
		break;
	case 'N':
		printf("\tN\t\e[1mEqualize no AUX\e[0m time_step[us] name\n");
		break;

	}
}

void printargs(char **argv){
	printf("Usage: %s option [params]\n", argv[0]);
	printarg('D');
	printarg('P');
	printarg('C');
	printarg('T');
	printarg('E');
	printarg('N');
}

int main(int argc, char **argv){
	if(argc < 2) {
		printargs(argv);
		exit(1);
	}
	switch (*argv[1]){
		case 'D':{
			if (argc < 2) {
				printarg('D');
				exit(1);
			}
			if (argc == 3 && CheckFiles(string(argv[2]))>0) exit(3);
			initialize();
			protocolLoadcellTest();
			if (argc == 3) ReportsSave(string(argv[2]));
			cleanup();
		}break;
		case 'P': {
			if (argc < 4) {
				printarg('P');
				exit(1);
			}
			if (atof(argv[2])<0.01){
				cout<<"v[mm/s]<=0.01!\n";
				exit(3);
			}
			if (argc == 5 && CheckFiles(string(argv[4]))>0) exit(3);
			initialize();
			protocolPremik(atof(argv[2]),atoi(argv[3]));
			if (argc == 5) ReportsSave(string(argv[4]));
			cleanup();
		}break;
		case 'C': {
			if (argc < 5) {
				printarg('C');
				exit(1);
			}
			if (atof(argv[2])*2*abs(atoi(argv[3]))<0.01){
				cout<<"v[mm/s]<=0.01!\n";
				exit(3);
			}
			if (argc == 6 && CheckFiles(string(argv[5])) >0) exit(3);
			initialize();
			protocolPreconditioning(atof(argv[2]),atoi(argv[3]),atoi(argv[4]));
			if (argc == 6) ReportsSave(string(argv[5]));
		}break;
		case 'T': {
			if (argc < 5) {
				printarg('T');
				exit(1);
			}
			if (atof(argv[2])<0.01){
				cout<<"v[mm/s]<=0.01!\n";
				exit(3);
			}
			if (argc == 6 && CheckFiles(string(argv[5]))>0 ) exit(3);
			initialize();
			protocolTest(atof(argv[2]),atoi(argv[3]),atoi(argv[4]));
			if (argc == 6) ReportsSave(string(argv[5]));
		}break;
		case 'E': {
			if (argc!=4) {
				printarg('E');
				exit(1);
			}
			if (CheckFiles(string(argv[3]))==3) protocolEqualize(atoi(argv[2]),string(argv[3]));
		}break;
		case 'N': {
			if (argc!=4) {
				printarg('E');
				exit(1);
			}
			if (CheckFiles(string(argv[3]))==3) protocolEqualizeNotrigger(atoi(argv[2]),string(argv[3]));
		}break;
		default: printargs(argv);
		break;
	}
	gpioTerminate();
	return 0;
}