THIS IS RELATED FILE ONE ETH_IP NUCLEO H755ZIQ BOARD I
ASSIGN THE IP ADDRESS IN DYNAMICALLY WITHOUT STATIC
https://github.com/stm32-hotspot/STM32H7-LwIP-Examples/tree/main?tab=readme-ov-file#memory-layout  REFER THIS RELATED TO ETHERNET PROJECT 


My eth-ip file code works for some time, but I am not sure why it works. Initially, I thought it was because of caching, but I realized I was absolutely wrong. So, I worked on the eth-dhcp file again. I discovered that it is all based on how you assign memory for Ethernet and how you protect that memory using the Memory Protection Unit (MPU).

Since this MCU has two cores, I suggest removing CM4 from memory assignment initially, because CM7 handles Ethernet functionality with higher performance. Assign the RAM2 address for Ethernet, and use the remaining memory in RAM2 for lwIP.

(This was my suggestion) 

