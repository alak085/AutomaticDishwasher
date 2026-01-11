// Required for Servo control on ESP32
#include <ESP32Servo.h>
#include <math.h>

Servo Servo0;
Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;
Servo Servo10;

#define RXD_PIN 18
#define TXD_PIN -1   // not used

const int btnPin = 0;
const int servo_pinA0 = 7;
const int servo_pinA1 = 8;
const int servo_pinA2 = 9;
const int servo_pinA3 = 10;
const int servo_pinA4 = 11;
const int servo_pinA5 = 37;
const int led_pin = 13;
const int led_pin_external = 2;

static int readAngleA0Prev = 0, readAngleA1Prev = 0, readAngleA2Prev = 0;
static int readAngleA3Prev = 0, readAngleA4Prev = 0, readAngleA5Prev = 0;
static int readAngleA0 = 0, readAngleA1 = 0, readAngleA2 = 0;
static int readAngleA3 = 0, readAngleA4 = 0, readAngleA5 = 0;

static int j = 0;
const double pi = 3.14159265359;

#define EPSILON 1e-9


typedef struct 
{
    double s1x;
    double s2y;
    double hypotenuse;

}Triangle;



double cleanAlmostZero(double val) {
    if (fabs(val) < EPSILON) {
        return 0.0;
    }
    if (val == 0.0) {
        return 0.0;
    }
    return val;
}

void print2DDoubleArray(double** arr, int rows, int cols) {
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            printf("%lf ", *(*(arr + i) + j));
        }
        printf("\n");
    }
    printf("\n");
    printf("\n");
}

double** makeMulArr(double theta, double alpha, double d, double a)
{
    double** myArr;
    myArr = (double**)malloc(sizeof(double*) * 4);
    for(int i = 0; i < 4; i++)
    {
        myArr[i] = (double*)malloc(sizeof(double) * 4);
    }

    theta = theta * (M_PI / 180.0);
    alpha = alpha * (M_PI / 180.0);

    myArr[0][0] = cos(theta);
    myArr[0][1] = -sin(theta) * cos(alpha);
    myArr[0][2] = sin(theta) * sin(alpha);
    myArr[0][3] = a * cos(theta);

    myArr[1][0] = sin(theta);
    myArr[1][1] = cos(theta) * cos(alpha);
    myArr[1][2] = -cos(theta) * sin(alpha);
    myArr[1][3] = a * sin(theta);

    myArr[2][0] = 0;
    myArr[2][1] = sin(alpha);
    myArr[2][2] = cos(alpha);
    myArr[2][3] = d;

    myArr[3][0] = 0;
    myArr[3][1] = 0;
    myArr[3][2] = 0;
    myArr[3][3] = 1;

    return myArr;

}

double** ansDHarr_setUp()
{
    double** myArr;
    myArr = (double**)malloc(sizeof(double*) * 4);
    for(int i = 0; i < 4; i++)
    {
        myArr[i] = (double*)malloc(sizeof(double) * 4);
    }

    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            myArr[i][j] = 0;
        }
    }
    myArr[0][0] = 1;
    myArr[1][1] = 1;
    myArr[2][2] = 1;
    myArr[3][3] = 1;
    return myArr;
}

void freeAns(double** myArr)
{
    
    for(int i = 0; i < 4; i++)
    {
        free(myArr[i]);
    }
    free(myArr);
}

double** multiplyMatricesPtr(double** a, double** b) 
{
        

        const uint32_t a_rows = 4;
        const uint32_t a_cols = 4;
        const uint32_t b_cols = 4;
        double firstVal;
        double secondVal;
        
        double** c;

        if (a == NULL || b == NULL || a[0] == NULL || b[0] == NULL)
        {
            return NULL;
        }

        c = (double**)malloc(a_rows * sizeof(double*));
        for(int i = 0; i < a_rows; i++)
        {
            c[i] = (double*)malloc(b_cols * sizeof(double));
        }

        for(int j = 0; j < a_rows; j++)
        {
            for(int k = 0; k < b_cols; k++)
            {
                double total = 0.0;
                for(int l = 0; l < a_cols; l++)
                {
                    firstVal = a[j][l];
                    firstVal = cleanAlmostZero(firstVal);
                    secondVal = b[l][k];
                    secondVal = cleanAlmostZero(secondVal);
                    total += (firstVal * secondVal);
                }

                c[j][k] = total;
            }
        }
        for (int i = 0; i < a_rows; i++) {
            for (int j = 0; j < b_cols; j++) {
                a[i][j] = c[i][j];
            }
        }


        for (int i = 0; i < a_rows; i++) 
        {
            free(c[i]);
        }

        free(c);

        return a;
}

double** create_DHarrTable()
{
    double** myArr;
    myArr = (double**)malloc(sizeof(double*) * 6);
    for(int i = 0; i < 6; i++)
    {
        myArr[i] = (double*)malloc(sizeof(double) * 4);
    }
    return myArr;
}

void free_DHarrTable(double** myArr)
{
    for(int i = 0; i < 6; i++)
    {
        free(myArr[i]);
    }
    free(myArr);
}

void addData(double** myArr, double theta0, double theta1, double theta2, double theta3, double theta4, double theta5)
{
    myArr[0][0] = theta0;
    myArr[0][1] = -90;
    myArr[0][2] = 90;
    myArr[0][3] = 48.68;

    myArr[1][0] = theta1;
    ;
    myArr[1][1] = 180;
    myArr[1][2] = 0;
    myArr[1][3] = 113.6919;

    myArr[2][0] = -theta2;
    myArr[2][1] = 90;
    myArr[2][2] = 0;
    myArr[2][3] = 98.625;

    myArr[3][0] = theta3;
    myArr[3][1] = -90;
    myArr[3][2] = 33.394;
    myArr[3][3] = 0;

    myArr[4][0] = theta4;
    myArr[4][1] = 90;
    myArr[4][2] = 0;
    myArr[4][3] = 32.526;

    myArr[5][0] = theta5;
    myArr[5][1] = 0;
    myArr[5][2] = 0;
    myArr[5][3] = 0;
}



double** R06_negates(double** myArr)
{
    myArr[0][0] = 1;
    myArr[0][1] = 0;
    myArr[0][2] = 0;
    myArr[0][3] = 0;

    myArr[1][0] = 0;
    myArr[1][1] = 1;
    myArr[1][2] = 0;
    myArr[1][3] = 0;

    myArr[2][0] = 0;
    myArr[2][1] = 0;
    myArr[2][2] = 1;
    myArr[2][3] = 36.250;

    myArr[3][0] = 0;
    myArr[3][1] = 0;
    myArr[3][2] = 0;
    myArr[3][3] = 1;

    return myArr;
}

double** R06_negate(double** myArr, double xEnd, double yEnd, double zEnd, double x, double y, double z)
{
    double three = xEnd * (M_PI / 180.0);
    double two = yEnd * (M_PI / 180.0);
    double one = zEnd * (M_PI / 180.0);

    myArr[0][0] = cos(one) * cos(two);
    myArr[0][1] = cos(one) * sin(two) * sin(three) - sin(one) * cos(three);
    myArr[0][2] = cos(one) * sin(two) * cos(three) + sin(one) * sin(three);
    myArr[0][3] = x;

    myArr[1][0] = sin(one) * cos(two);
    myArr[1][1] = sin(one) * sin(two) * sin(three) + cos(one) * cos(three);
    myArr[1][2] = sin(one) * sin(two) * cos(three) - cos(one) * sin(three);
    myArr[1][3] = y;

    myArr[2][0] = -sin(two);
    myArr[2][1] = cos(two) * sin(three);
    myArr[2][2] = cos(two) * cos(three);
    myArr[2][3] = z;

    myArr[3][0] = 0;
    myArr[3][1] = 0;
    myArr[3][2] = 0;
    myArr[3][3] = 1;

    return myArr;
}

double radians_to_degrees(double input)
{
    return input * 180 / pi;
}

double law_of_cosines(double s1, double s2, double s3)
{
    double temp = pow(s1,2) + pow(s2,2) - pow(s3, 2);
    double temp1 = temp/(2*s1*s2);
    double ans = acos(temp1);
    return ans;
}

void InverseKinmatics0to3(double x, double y, double z, double* theta0, double* theta1, double* theta2)
{
    const double s1 = 113.6919;
    const double s2 = 105;
    const double xPntA = 48.68;
    const double yPntA = 90;
    double xPntB = sqrt(x * x + y * y);
    double yPntB = z;

    double angleOffset = 0;
    double theta0Offset = 0;
    double mode = 0;
    double theta1Part1;

    
    if(z > 90)
    {
        
        mode = 1;
        angleOffset = 90;
    }

    Triangle rightTri;

    rightTri.s1x = fabs(xPntB - xPntA);
    rightTri.s2y = fabs(yPntB - yPntA);

    double temp = pow(rightTri.s1x, 2) + pow(rightTri.s2y, 2);
    rightTri.hypotenuse = sqrt(temp);

    

    double myTheta0 = radians_to_degrees(atan2(y,x));
   
    *theta0 = myTheta0;

    if(mode)
    {
        theta1Part1 = radians_to_degrees(law_of_cosines(rightTri.s1x, rightTri.hypotenuse, rightTri.s2y));
    }
    else
    {
        theta1Part1 = radians_to_degrees(law_of_cosines(rightTri.s2y, rightTri.hypotenuse, rightTri.s1x));
    }
    printf("x %lf\n", rightTri.s1x);
    printf("y %lf\n", rightTri.s2y);
    printf("hypot %lf\n", rightTri.hypotenuse);
    double theta1Part2 = radians_to_degrees(law_of_cosines(s1, rightTri.hypotenuse, s2));
    printf("theta1Part1 %lf theta1Part2 %lf\n", theta1Part1, theta1Part2);
    *theta1 = theta1Part1 + theta1Part2 + angleOffset;
    double realTheta2 = 180 - radians_to_degrees(law_of_cosines(s1, s2, rightTri.hypotenuse));
    printf("realTheta2 %lf\n", realTheta2);
    *theta2 = realTheta2;

    printf("%lf\n", myTheta0 + theta0Offset);
    printf("%lf\n", theta1Part1 + theta1Part2 + angleOffset);
    printf("%lf\n", realTheta2);

    printf("finished\n");

}

void InverseKinmatics4to6(double** myArr, double* theta4, double* theta5, double* theta6)
{
 *theta5 = atan2( sqrt(1.0 - myArr[2][2]*myArr[2][2]), myArr[2][2]) * 180 / M_PI;
 *theta4 = atan2( myArr[0][2], myArr[1][2] ) * 180 / M_PI;
 *theta6 = atan2( -myArr[2][0], myArr[2][1] ) * 180 / M_PI;;
}

// Inverts a 4x4 transformation matrix (composed of a 3x3 rotation and 3x1 translation)
double** invertTransformMatrix(double** matrix) {
    // Allocate memory for the new inverted matrix
    double** inverted = ansDHarr_setUp(); // Re-uses your allocation function

    // Transpose the 3x3 rotation part
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            inverted[i][j] = matrix[j][i];
        }
    }

    // Calculate the new translation part: -R^T * p
    double p[3] = { matrix[0][3], matrix[1][3], matrix[2][3] };
    inverted[0][3] = -(inverted[0][0] * p[0] + inverted[0][1] * p[1] + inverted[0][2] * p[2]);
    inverted[1][3] = -(inverted[1][0] * p[0] + inverted[1][1] * p[1] + inverted[1][2] * p[2]);
    inverted[2][3] = -(inverted[2][0] * p[0] + inverted[2][1] * p[1] + inverted[2][2] * p[2]);

    // The last row is always [0, 0, 0, 1]
    inverted[3][0] = 0;
    inverted[3][1] = 0;
    inverted[3][2] = 0;
    inverted[3][3] = 1;

    return inverted;
}

void setup() {
  Serial.begin(115200);

  Serial.begin(115200);                // USB debug
  Serial2.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);  // UART receive
  Serial.println("Receiver ready...");

  Servo0.attach(servo_pinA0);
  Servo1.attach(servo_pinA1);
  Servo2.attach(servo_pinA2);
  Servo3.attach(servo_pinA3);
  Servo4.attach(servo_pinA4);
  Servo10.attach(servo_pinA5);

  pinMode(btnPin, INPUT_PULLDOWN);
  pinMode(led_pin, OUTPUT);
  
}

void loop() {
    // Read analog inputs for x, y, z
    int sensorValueA3 = analogRead(A3);
    int sensorValueA4 = analogRead(A4);
    int sensorValueA5 = analogRead(A5);

    // Map to 0 to 170 mm range (adjust as needed)
    int x = map(sensorValueA3, 0, 4095, -170, 170);
    int y = map(sensorValueA4, 0, 4095, 0, 170);
    int z = map(sensorValueA5, 0, 4095, -80, 200);

    double theta0STore, theta1STore, theta2STore;

    printf("\n\n x val %d \n\n", x);
    printf("\n\n y val %d \n\n", y);
    printf("\n\n z val %d \n\n", z);

    // ==========================================================
    //  1. ALLOCATE ALL MEMORY AT THE START
    // ==========================================================
    double** myArrTableVals = create_DHarrTable();
    double** answer = ansDHarr_setUp();
    double** ro6Negate = ansDHarr_setUp();
    double** ro6RevKin = ansDHarr_setUp();
    double** ro6RevKinCopy = ansDHarr_setUp();
    
    // ==========================================================
    //  2. PERFORM CALCULATIONS
    // ==========================================================
    R06_negates(ro6Negate);
    R06_negate(ro6RevKin, 0, 0, 0, x, y, z);
    multiplyMatricesPtr(ro6RevKin, ro6Negate); // Safe to call now

    R06_negate(ro6RevKinCopy, 0, 0, 0, x, y, z);
    multiplyMatricesPtr(ro6RevKinCopy, ro6Negate); // Safe to reuse ro6Negate

    InverseKinmatics0to3(ro6RevKin[0][3], ro6RevKin[1][3], ro6RevKin[2][3], &theta0STore, &theta1STore, &theta2STore);

    addData(myArrTableVals, theta0STore, theta1STore, theta2STore, 0, 0, 0);

    for (int i = 0; i < 3; i++) {
        double** temp = makeMulArr(myArrTableVals[i][0], myArrTableVals[i][1], myArrTableVals[i][2], myArrTableVals[i][3]);
        multiplyMatricesPtr(answer, temp);
        freeAns(temp); // Free the temporary matrix created inside the loop
    }

    multiplyMatricesPtr(answer, ro6RevKinCopy);
    
    double theta4, theta5, theta6;
    InverseKinmatics4to6(answer, &theta4, &theta5, &theta6);

    printf("Solution (Elbow-UP):\n");
    printf("theta1 = %f\n", theta0STore);
    printf("theta2 = %f\n", theta1STore);
    printf("theta3 = %f\n", theta2STore);
    printf("theta4 = %f\n", theta4);
    printf("theta5 = %f\n", theta5);
    printf("theta6 = %f\n\n", theta6);

    // ==========================================================
    //  3. FREE ALL MEMORY AT THE END
    // ==========================================================
    free_DHarrTable(myArrTableVals);
    freeAns(answer);
    freeAns(ro6RevKin);
    freeAns(ro6RevKinCopy);
    freeAns(ro6Negate);
    
    // ==========================================================
    //  4. UPDATE SERVOS AND HANDLE SERIAL
    // ==========================================================
    Servo0.write((int)theta0STore);
    Servo1.write((int)theta1STore);
    Servo2.write((int)theta2STore);
    Servo3.write((int)theta4);
    Servo4.write((int)theta5);
    Servo10.write((int)theta6);

    while (Serial2.available()) {
        char c = Serial2.read();
        Serial.write(c);
    }

    
}