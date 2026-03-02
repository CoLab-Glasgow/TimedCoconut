#ifndef MINIPID_H
#define MINIPID_H

class MiniPID{
public:
	MiniPID(double, double, double);
	MiniPID(double, double, double, double);
	__attribute__((noinline, used)) void setP(double);
	__attribute__((noinline, used)) void setI(double);
	__attribute__((noinline, used)) void setD(double);
	__attribute__((noinline, used)) void setF(double);
	__attribute__((noinline, used)) void setPID(double, double, double);
	__attribute__((noinline, used)) void setPID(double, double, double, double);
	__attribute__((noinline, used)) void setMaxIOutput(double);
	__attribute__((noinline, used)) void setOutputLimits(double);
	__attribute__((noinline, used)) void setOutputLimits(double,double);
	__attribute__((noinline, used)) void setDirection(bool);
	__attribute__((noinline, used)) void setSetpoint(double);
	__attribute__((noinline, used)) void reset();
	__attribute__((noinline, used)) void setOutputRampRate(double);
	__attribute__((noinline, used)) void setSetpointRange(double);
	__attribute__((noinline, used)) void setOutputFilter(double);
	__attribute__((noinline, used)) double getOutput();
	__attribute__((noinline, used)) double getOutput(double);
	__attribute__((noinline, used)) double getOutput(double, double);

private:
	double clamp(double, double, double);
	bool bounded(double, double, double);
	void checkSigns();
	void init();
	double P;
	double I;
	double D;
	double F;
	double maxIOutput;
	double maxError;
	double errorSum;
	double maxOutput; 
	double minOutput;
	double setpoint;
	double lastActual;
	bool firstRun;
	bool reversed;
	double outputRampRate;
	double lastOutput;
	double outputFilter;
	double setpointRange;
};
#endif