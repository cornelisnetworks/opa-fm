/*
**
**  VIEO SDK
**  Copyright (c) 2001, 2002, VIEO Inc.  All rights reserved.
**
**  utilities_README.txt for Release 2.1
**  $Revision: 1.4 $ -- $Date: 2002/11/19 16:31:54 $
**
*/

Instructions for qp1recv.c and qp1send.c sample programs.

COMPONENT BEING TESTED: Management API

PURPOSE: The qp1send.c sample program will send a LID-routed MAD using the SEND method.
The 'recv' program will receive the MAD and then send a reply. The qp1recv.c sample program will
wait 5 seconds for a message (MAD) to be received from the fabric. It will then print the message and exit.

CONFIGURATION:  System A (Linux configuration) and system B (Linux configuration).

SETUP: 	Connect system A and system B via port 5.

INSTRUCTIONS:

1. Find the LIDs of system A (<slid>) and system B (<dlid>) by executing the Subnet Manager (SM):

    Change to the bin directory on system A :

	# cd /<workdir>/vieo/SuSE72-4GB-SMP/bin

    Execute the following command to start the SM:

	#./sm

NOTE: It is not necessary for the Subnet Manager (SM) to continue running.

2. On system B, run the qp1recv.c program:

	# ./qp1recv 

3. Within 5 seconds, run the qp1send.c program on system A: 

	# ./qp1send <slid> <dlid>  

qp1send.c performs the following actions:

1. Get the SLID and DLID.
2. Open the log file.
3. Initialize the MAI subsystem and open the port.
4. Setup the data for a MAD.
5. Send the message.

qp1recv.c performs the following actions:

1. Open the log file.
2. Initialize the MAI subsystem and open the port. 
3. Create the filter. 
4. Wait five second for an answer.
5. Display the MAD that came in.
6. Delete the filter.
7. Dump the received data.
