# E-Voting v2
Authors: Abdelaziz Hussein, Sam Krasnoff, Yanni Pang

Date: 2020-11-13
-----

## Summary
A secure election system is very important for democracy. Without a secure way to cast a ballot, the result of an election could be compromised by fraud. To explore the concept of elections, we made a program to decide an elected leader via IR (Near field communication) and UDP socket exchanges. Each fob differs by ID, but are otherwise the same code. We create a secure near field communications link between two fobs via IR. Once a vote is passed to the fob, the result  propagated to a "poll leader" that will then send the result to a server.  

Here are some specifications that the program follows:  

- The code for each fob should be identical excluding DeviceID
- Original votes are to be communicated via IR channel from one fob to another.
- Once the vote has been received, it should be communicated to the Poll Leader
- The Poll Leader will send the results to a server (pi) which will log the vote to a database
- Poll results will be accessible via the server via http
- The web server will be able to do a complete restart of the vote
- The Poll Leader should be selected from your set of fobs
- If the Poll Leader fails, a new one should be selected

## Investigative Question  
List 5 different ways that you can hack the system (including influencing the vote outcome or preventing votes via denial of service). For each above, explain how you would mitigate these issues in your system.

- **Spoof UDP packet**
  - Using netcat, you can use a command like ```echo "malicious packet here" | nc -w1 -u pi.yanni.dev 3333``` to send a packet to the server. If the packet is formatted correctly, one could increase the votes for a certain fob.
  - To mitigate this issue, the packet could be encrypted using private and public keys, or a checksum can be generated to verify the integrity of the packet.
- **Invalid UDP packet**
  - If an invalid packet is sent to a server that does not catch any errors, a parsing function like JSON.parse() could throw a SyntaxError and crash the server.
  - To mitigate this issue, be sure to test the server for functionality that disregards any incorrect input.
- **DDOS**
  - If a great magnitude of packets are sent to the server, it could cause the server to use up too much of its resources causing it to crash or rendering it unable to receive other legitimate packets.
  - To mitigate this issue, one could implement some code to only accept a certain amount of packets at a time and reply to the poll leader with a successful receipt of a packet.
- **IR Spoofing**
  - If another device such as another ESP32 or another device with IR capability is pointed to the fobs, it may cause unexpected behavior such as crashing or being unable to read the other fobs. Specifically, another ESP32 could be made to spoof a packet to influence the votes.
  - To mitigate this issue, a checksum could be used to verify received packets, and or encryption can be used. To mitigate the IR DDOS issue, an better secure method of connection can be used such as an ethernet cable.
- **Physical Security**
  - Physical security is also important. If the fobs are left on a table, malicious users could reset the fobs, trigger more votes, or completely disconnect them from power. 
  - To mitigate this issue, the fobs could be locked inside a safe, or a box to prevent tampering.

## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Fob performs IR NFC data exchange of vote to another fob (LED indication) | 1 |  1     | 
| Receiving fob communicates vote to Poll Leader (LED indication) via network communication | 1 |  1     | 
| Poll leader (LED indication) is replaced if fails | 1 |  1     | 
| Poll Leader reports votes to server database. | 1 |  1     | 
| Portal allows query to database to show actual vote counts per candidate | 1 |  1     | 
| Operates over mutiple sites or with all available fobs (up to 9) | 1 |  1     | 
| Investigative question response | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design
Our approach to the design of this program:  

- **Leader selection algorithm**  
  -The ESP that is turned on first, sends data packets via UDP and waits a response, if no response is recieved within 5 seconds, the ESP initiates the election.
  - Once the two other ESPs have been turned on, They recieve the data packets from the first ESP and acknowledge that the election has already begun.
  - The two ESPs compare the min val (the ID of the ESP which started the election) recieved to their IDs and send back to the Leader a data packet with their IDs and the new       min val.
  - Once the starter of the election recieves the two data packets with the updated min val, it compares the two recieved minvals, chooses the smallest in value, and compares it     to the ID of all the ESPs including itself.
  - Once it determines the fob with the smallest ID, the starter of the election acknowledges who the leader is and sends a data packet to both ESPs then they acknowledge who       the new leader is as the minval they recieved is the correct min val. 
  
- **Complete the IR-TX/RX Skill: build fobs and bring up TX/RX code to demonstrate it works: Adapt code to transmit vote (R, B, G) using IR.**
  - The UART payload sent via IR is formatted as -- ``` | fobID |  votedFor  |``` 
  - Outputs 38kHz using RMT (remote control) for IR transmission
  - Onboard LED blinks device ID (myID)
  - Button press to initiate vote
  - RGB LED shows vote

- **Set up UDP message passing on your local wireless network: build a method to exchange vote payloads to one or more destinations (e.g., point-to-point or point-multipoint). Make sure it works from fob to leader and leader to node.js**
  - The UDP payload sent via sockets is formatted as -- | fobID |  votedFor  |
  - Secure payload is sent to the fob leader using UDP sockets on port 3333 to a specified fob via IP address
  - Contains a function to continually read any incoming messages
  - Contains a function to send a packet to specified host via port 3333
  - Poll leader sends a formatted packet in JSON format to a remote node.js server (pi.yanni.dev) listening on port 3333.
    - JSON packet: 
    ``` 
    {
     "fobID": %d,
     "votedFor": %d
    } 
    ```

- **Complete the Database Skill: set up tingodb or equivalent on your server for the vote data and be able to receive and save data from the leader to the node.js and to the DB.**  
  - Node.js server uses the following libraries:
    - **express** to deliver the client-facing HTML page.
    - **http** to listen on specified port to host site (8080)
    
    - **dgram** to receive JSON packet sent via UDP socket from the poll leader
    - **level** to create and store elements in a database
    - **socket.io** to:
      - receive a command from client-facing page to clear database and reset vote count
      - send a formatted packet to the client-facing page via sockets
        - JSON Packet: 
```
{
  fobID: id,
  time: time,
  vote: vote,
  fob1Total: fob1,
  fob2Total: fob2,
  fob3Total: fob3
}
```

- **Interconnect your node.js server to the DB and host the queries for the vote data on a web page.**
  - HTML and Javascript was used to create the client-facing web page.
  - On receipt of a packet delivered via socket.io:
    - a table is dynamically created based on the fobID received and votedFor data
    - a running total of the amount of votes received is tracked
    - chart.js is used to track the total number of votes

- **Integrate these steps into as single application that is loaded onto all of the fobs.**  




## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>

![image](https://github.com/BU-EC444/Team10-Hussein-Krasnoff-Pang/blob/master/quest-4/images/Screenshot%202020-11-13%20at%201.46.08%20PM.png)
![image](https://github.com/BU-EC444/Team10-Hussein-Krasnoff-Pang/blob/master/quest-4/images/Unknown.jpeg)
![image](https://github.com/BU-EC444/Team10-Hussein-Krasnoff-Pang/blob/master/quest-4/images/e-vote-fsm.png)

## Supporting Artifacts
- [Link to Solution Design/overview](https://photos.app.goo.gl/t6doJjThfeDf5GhPA). 
- [Link to Demo](https://photos.app.goo.gl/J1zXf2uD4g3CHN1t9)


## Modules, Tools, Source Used Including Attribution

- IR/TX Example Code
- Database Example Code
- UDP client/server example code

**Nodejs Modules**
- express
- http
- dgram
- levelDB
- socket.io

##References
-----

