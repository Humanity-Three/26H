from machine import FPIOA, UART
import time


UART_BAUD = 115200
UART_TX_PIN = 50
UART_RX_PIN = 51

fpioa = FPIOA()
fpioa.set_function(UART_RX_PIN, FPIOA.UART3_RXD)
fpioa.set_function(UART_TX_PIN, FPIOA.UART3_TXD)
uart = UART(UART.UART3, UART_BAUD)

counter = 0
print("UART3 test: PIN50 TX, PIN51 RX, 115200")

while True:
    message = "DX:%d,DY:0,A0,D1\r\n" % counter
    written = uart.write(message.encode())
    print("wrote=%s %s" % (str(written), message.strip()))
    counter = (counter + 1) % 1000
    time.sleep_ms(100)
