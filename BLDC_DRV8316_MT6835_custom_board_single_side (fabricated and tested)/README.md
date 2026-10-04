# BLDC driver DRV8316, MT6835

Custom PCBA build BLDC driver based on DRV8316 and absolute encoder MT6835, which allows for driving a BLDC already. There is also a CAN transceiver SN65HVD230DR on board for conduction CAN communication.


This is a slightly bigger version that was actually fabricated is also more economical due to 1-sided SMD assembly:

<img width="248" height="286" alt="image" src="https://github.com/user-attachments/assets/50c3ea64-6302-4c30-b269-bb544ed0e9e3" />

Soldering on 2 cables; 1 for the main power supply 8-24 V, and the other for the 3 phase BLDC 

<img width="1651" height="895" alt="image" src="https://github.com/user-attachments/assets/8db0894d-9f7c-4f4c-9616-4c0888c42071" />

Board with BLDC GB2806 mounted, along with a 6 mm diametric magnet mounted directly on motor shaft for position feedback 

<img width="1155" height="757" alt="image" src="https://github.com/user-attachments/assets/8e7a8cd5-ab9c-426f-9beb-ecb82d087f0e" />

The magnet is on the opposite of the MT6835 absolute encoder package, and is about 1 mm above the PCBA

<img width="732" height="298" alt="image" src="https://github.com/user-attachments/assets/f95a3e84-a90a-4e94-954b-765c846e3238" />


3 boards connected over CAN bus. 2 of these boards have solder bridge (jumper) for CAN termination. 
<img width="1319" height="686" alt="image" src="https://github.com/user-attachments/assets/9055fb1c-4d24-4ab4-82ce-ef7f1553c305" />


<img width="1040" height="1087" alt="image" src="https://github.com/user-attachments/assets/e4c1addd-0977-407a-a8f6-d88b781dc2b6" />


<img width="754" height="793" alt="image" src="https://github.com/user-attachments/assets/8ef104e7-0e3f-4e91-a81a-9b1b6230ce2c" />

