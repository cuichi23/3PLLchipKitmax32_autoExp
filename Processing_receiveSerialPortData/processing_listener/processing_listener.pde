import processing.serial.*;

Serial myPort;        // Create object from Serial class
String val;           // Data received from the serial port
PrintWriter output;

void setup()
{
  // I know that the first port in the serial list on my mac is Serial.list()[0].
  // On Windows machines, this generally opens COM1.
  // Open whatever port is the one you're using.
  String portName = Serial.list()[0]; //change the 0 to a 1 or 2 etc. to match your port
  myPort = new Serial(this, portName, 115200);
  output = createWriter("measurements2018.txt");
}

void draw()
{
  if ( myPort.available() > 0) {                // if data is available,
    val = myPort.readStringUntil('\n');         // read it and store it in val
  } 
  else {                                        // do nothing in that case
  }
  
  output.print(val);
  //println(val); //print it out in the console
}

void keyPressed() {
  output.flush();  // Writes the remaining data to the file
  output.close();  // Finishes the file
  exit();  // Stops the program
}
