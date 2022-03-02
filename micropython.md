# Micropython


## Basics

```python
# Izdrukāt ziņu
print('Hello World!')
```

## Time

```python
# Pievienot time moduli
import time

# Gaidīt
time.sleep(1)       # 1 second
time.sleep_ms(10)   # 10 milliseconds
time.sleep_us(50)   # 50 microseconds

# Nolasīt relatīvu pulksteni milisekundēs
time.ticks_ms()
# Nolasīt relatīvu pulksteni mikrosekundēs
time.ticks_us()

# Noteikt starpību starp laika punktiem
time.ticks_diff(t1, t2)
```


## GPIO - General Purpose Input/Output (Vispārīga pielietojuma Ieejas/Izejas)

```python
# Pievienot Pin klasi (moduli)
from machine import Pin

# --- Izeja ---
# Izveidot izejas kāju
p0 = Pin(0, Pin.OUT)

# Uzstādīt zemu (0) un pēc tam augstu līmeni (1)
p0.value(0)
p0.off()

p0.value(1)
p0.on()

# Nomainīt izejas stāvokli uz pretējo
p0.toggle()


# --- Ieeja ---
# Izveidot ieejas kāju
p1 = Pin(1, Pin.IN)

# Iveidot ieejas kāju ar pull-up rezistoru
p2 = Pin(2, Pin.IN, Pin.PULL_UP)

# Nolasīt līmeni uz kājas un izdrukāt
print(p2.value())

# Interrupt (Pārtraukums) - uz frontes (edge) tiek izpildīta funkcijas
# IRQ_FALLING - krītošā fronte (fallng edge), 1 -> 0
# IRQ_RISING - augošā fronte (rising edge), 0 -> 1
# IRQ_BOTH - abas frontes
p2.irq(isrFunc, Pin.IRQ_FALLING)

def isrFunc(pin):
  print('IRQ')

# vai izmantojot lambda funkciju
p2.irq(lambda pin: print('IRQ'), Pin.IRQ_FALLING)

```

## PWM - Pulse Width Modulation (Pulsa Platuma Modulācija)

```python
# Pievienot PWM un Pin klases (moduļus)
from machine import Pin, PWM

# Izveidot PWM izeju
pwm0 = PWM(Pin(0))

# Uzstādīt PWM frekvenci
pwm0.freq(1000)

# Uzstādīt PWM platumu (duty cycle)
pwm0.duty_u16(32767)  # Vērtība robežās no 0 līdz 65535
```

## ADC - Analog to Digital Converter (Analogais Ciparu Pārveidotājs)

```python
# Pievienot ADC un Pin klases (moduļus)
from machine import Pin, ADC

# Izveidot ADC objektu 0.kanālam (kāja 26)
adc = ADC(0)

# Nolasīt vērtību 0 - 65535, kas atbilst 0V - 3.3V
adc.read_u16()

# ADC rezultāts jāreizina ar 3.3/65535, lai iegūtu spriegumu uz ADC kājas
voltage = 3.3 / 65535 * adc.read_u16()
```

## Timer

Timers ļauj izpildīt funkciju ar regulāru intervālu vai vienreizēji pēc noteikta laika

```python
# Pievienot Timer moduli (parauga nolūkiem arī Pin)
from machine import Timer

# Funkcija, ko vēlamies periodiski izpildīt
def periodicFunc(timer):
  print('Tick')

# Izveidot Timer objektu
tim = Timer()

# Inicializēt Taimeri ar 1s (1 Hz) periodu
tim.init(freq=1.0, mode=Timer.PERIODIC, callback=periodicFunc) 

# Inicializēt Timeri, kas izpildīsies vienu reizi pēc 200ms + lambda fukcija
tim.init(period=200, mode=Timer.ONE_SHOT, callback=lambda t: print('Mjau'))
```

## UART - Universal Asynchrounos Receiver Transmitter

Raspberry Pi Pico ir divi UART - UART0 un UART1.

```python
# Pievienot UART un Pin klases (moduļus)
from machine import Pin, UART

# Izveidot UART objektu pievienotu pie kājām 16 un 17, ar komunikācijas ātrumu 19200 
uart0 = UART(0, tx=Pin(16), rx=Pin(17), baudrate=19200)

# Nosūtīt ziņu 'Hello'
uart0.write('Hello')

# Nolasīt 5 saņemots bytes
uart0.read(5)

# Nolasīt visus saņemtos bytes
uart0.read()

# Noskaidrot, cik bytes jau saņemti
uart0.any()


```

## I2C

```python
# Pievienot I2C un Pin klasi (moduli)
from machine import I2C, Pin

# Izveidot I2C objektu uz kājām 16 un 17 ar ātrumu 100kHz
i2c0 = I2C(0, sda=Pin(16), scl=Pin(17), freq=100000)

# Skenēt pievienotās I2C iekārtas
i2c0.scan() # Atgriež sarakstu ar pievienoto iekārtu adresēm

# Nolasīt 4 bytes no iekārtas ar adresi 0x3A
i2c0.readfrom(0x3A, 4)

# Nosūtīt '12' iekārtai ar adresi 0x3A
i2c0.writeto(0x34, '12')
```

## NeoPixel

```python
# Pievienot Pin un neopixel klases
from machine import Pin
from neopixel import NeoPixel

# Izveidot neopixel opjektu ar 3 virknē saslēgtiem LED
np = NeoPixel(Pin(0, Pin.OUT), 3)

# Uzstādīt pirmo LED baltu (Red, Green, Blue) - RGB vērtības no 0 līdz 255
np[0] = (255, 255, 255)

# Nosutīt LED vērtiības
np.write()

# Iegūt pirmā LED RGB vērtības
r, g, b = np[0]
```