void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}
int t=0;
void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(t);
  //Serial.println(",101.123456,58.123456,0,-1,1,180,25,100,50,1,85");
  String mm = ",101.123456,58.123456";
  mm += ','+String(random(-1,2))+','+String(random(-1,2))+','+String(random(-1,2));
  mm += ','+String(random(-180,180))+','+String(random(20,40))+','+String(random(0,100))+','+String(random(0,100));
  mm += ','+String(random(0,1))+','+String(random(80,90))+','+String(random(-180,180))+','+String(random(-180,180));
  Serial.println(mm);
  
  t+=1;
  delay(1000);
}
