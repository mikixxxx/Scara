
    
  
import time
from rho4 import Rho4

HOST = "192.168.4.1"
PORT = 6051




def main():
    rho = Rho4(HOST, PORT, timeout=3.0)
    rho.connect()

    try:
        print("PING:", rho.ping())
        

    finally:
        rho.disconnect()


if __name__ == "__main__":
    main()

