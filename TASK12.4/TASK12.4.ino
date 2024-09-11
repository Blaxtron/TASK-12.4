using namespace std;
const int INPUT = A0;
double last_time;
double output;

float smoothing_factor;
double current_smoothed;
double previous_smoothed = 0;




class PID{
    private:
    float Kp;
    float Ki;
    float Kd;
    float goal; //set point
    float error; //e(t)
    float previousError;
    float dt;  //change in time
    double integral = 0;

    public:
    PID(float Kp,float Ki,float Kd):Kp(Kp),Ki(Ki),Kd(Kd){

    }
    
    void setGoal(float goal){
      this->goal = goal;
    }
    
    void setDt(float dt){
      this->dt = dt;
    }
    void setError(float error){
      this->error = error;
    }
    
    float getGoal(){
      return goal;
    }
    float getDt(){
      return dt;
    }
    float getError(){
      return error;
    }
    double calculation(double error){
      integral += error * dt;
      double derivative = (error-previousError)/dt;
      previousError = error;
      double ut = (Kp * error) + (Ki *integral) + (Kd * derivative); 
      return ut;
    }


};

PID myPID = PID(0.8,0.2,0.001);


void setup() {
  // put your setup code here, to run once:
  myPID.setGoal(200);
  
  Serial.begin(9600);

  last_time=0;
  smoothing_factor = 0.9; //setting the exponential filter smoothing factor
}

void loop() {
  // put your main code here, to run repeatedly:
  double elapsed = millis(); //elapsed time in milliseconds
  double reading = map(analogRead(INPUT),0,1024,0,255); //reading of the analog pin
  myPID.setError(myPID.getGoal()-reading); //error = goal - reading


  myPID.setDt(elapsed - last_time); //dt in milliseconds
  last_time = elapsed;
  output = myPID.calculation(myPID.getError()); //generates the output
  current_smoothed = smoothing_factor * output +(1-smoothing_factor)*previous_smoothed;
  previous_smoothed = current_smoothed;

  Serial.println(current_smoothed); //prints the current smoothed value
}
