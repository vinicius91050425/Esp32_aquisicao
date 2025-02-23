int Valor; // Leitura do ADC 
int offset = 102; // Ajuste de pressão zero 
int fullScale = 922; // Ajuste para pressão máxima (span) 
float pressaoSensor; 
float pressao; // Pressão final 

void setup() {

  Serial.begin(9600); // Porta Serial para comunicação com o ESP32
}

void loop() {
  Valor = analogRead(A0);
  
  // Conversão da pressão para PSI 
  pressaoSensor = ((float)(Valor - offset) * 150.0) / (fullScale - offset);  
  pressao = 1.0735 * pressaoSensor - 1.0963;

  if (pressao > 0) {
    
    Serial.print(pressao, 2); // Envia o valor da pressão para o ESP32
    Serial.println();
    
  } else {
    Serial.println("0,00");

  }

  delay(10000); // Atraso de 1 segundo 
}
