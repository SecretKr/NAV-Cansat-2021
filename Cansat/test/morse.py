import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
import time
letters = {
      'A': '.-', 
      'B': '-...', 
      'C': '-.-.', 
      'D': '-..', 
      'E': '.', 
      'F': '..-.', 
      'G': '--.', 
      'H': '....', 
      'I': '..', 
      'J': '.---', 
      'K': '-.-', 
      'L': '.-..', 
      'M': '--', 
      'N': '-.', 
      'O': '---', 
      'P': '.--.', 
      'Q': '--.-', 
      'R': '.-.', 
      'S': '...', 
      'T': '-', 
      'U': '..-', 
      'V': '...-', 
      'W': '.--', 
      'X': '-..-', 
      'Y': '-.--', 
      'Z': '--..', 
      '1': '.----', 
      '2': '..---', 
      '3': '...--', 
      '4': '....-', 
      '5': '.....', 
      '6': '-....', 
      '7': '--...', 
      '8': '---..', 
      '9': '----.', 
      '0': '-----', 
      ' ': '/'
          }
GPIO.setup(26, GPIO.OUT)
PIN = 26
u = 0.1
word = str(input(""))
word = list(word.upper())
def led_control(time_sleep):
    GPIO.output(PIN, 1)
    time.sleep(time_sleep)
    GPIO.output(PIN, 0)
    time.sleep(0.05)

for letter in word:
    time.sleep(2*u)
    bin_letter = letters[letter]
    print(bin_letter)
    for codes in bin_letter:
        if codes == "-":
            led_control(3*u)
        elif codes == ".":
            led_control(u)
        elif codes == "/":
            time.sleep(10*u)