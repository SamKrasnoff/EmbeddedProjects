//Yanni Pang Sam Krasnoff Abd el Aziz Hussein 11/13/20

// Modules
var level = require('level');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');

var PORT = 3333;
var HOST = '10.0.1.152';
var remoteport;
var remoteadd;
var dataReceived;
var server = dgram.createSocket("udp4");

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);

});

// Create or open the underlying LevelDB store
var db = level('./mydb', {valueEncoding: 'json'});
let i = 0;
let value;
let fob1 = 0;
let fob2 = 0;
let fob3 = 0;
server.on('message', function (message, remote) {
    try {
        var dataReceived = JSON.parse(message);
        var d = new Date();
        var time = d.getHours() + ":" + d.getMinutes() + ":" + d.getSeconds();
        let id = dataReceived.fobID;
        let vote = dataReceived.votedFor;
        switch(vote){
            case 1:
                fob1++;
                break;
            case 2:
                fob2++;
                break;
            case 3:
                fob3++;
                break;
        }
        value = [{fobID: id, time: time, vote: vote, fob1Total: fob1, fob2Total: fob2, fob3Total: fob3}];

        db.put([i], value, function (err) {
            if (err) return console.log('Ooops!', err) // some kind of I/O error
        });
        var msg = {[i]: value};
        // console.log("Added: " + Object.keys(msg));
        io.emit('message', msg);

        remoteport = remote.port;
        remoteadd = remote.address;
        console.log(dataReceived);
        // server.send(dataReceived.toString(), remote.port, remote.address, function (error) {
        //     if (error) {
        //         console.log('MEH!');
        //     } else {
        //         console.log('Sent: Ok!');
        //     }
        // });

        i++;
    } catch(e) {
        console.log(e);
    }


});
server.bind(PORT, HOST);


// Points to index.html to serve webpage
app.get('/', function (req, res) {
    res.sendFile(__dirname + '/index.html');
});

// Function to stream from database
function readDB(arg) {
    db.createReadStream()
        .on('data', function (data) {
            console.log(data.key, '=', data.value);
            // Parsed the data into a structure but don't have to ...
            var dataIn = {[data.key]: data.value};
            // Stream data to client
            io.emit('message', dataIn);
        })
        .on('error', function (err) {
            console.log('Oh my!', err)
        })
        .on('close', function () {
            console.log('Stream closed')
        })
        .on('end', function () {
            console.log('Stream ended')
        })
}

// When a new client connects
var clientConnected = 0; // this is just to ensure no new data is recorded during streaming
io.on('connection', function (socket) {

    console.log('a user connected');
    clientConnected = 0;

    // Call function to stream database data
    readDB();
    clientConnected = 1;
    socket.on('disconnect', function () {
        console.log('user disconnected');
    });
    socket.on('reset', function (message) {

        // server.send("1",remoteport,remoteadd,function(error){
        //     if(error){
        //         console.log('MEH!');
        //     }
        //     else{
        //         console.log('Sent:', message);
        //     }
        // });
        db.clear();
        fob1 = 0;
        fob2 = 0;
        fob3 = 0;
        console.log("db cleared");
    });
});

// Listening on localhost:3000
http.listen(8080, function () {
    console.log('listening on *:8080');
});


