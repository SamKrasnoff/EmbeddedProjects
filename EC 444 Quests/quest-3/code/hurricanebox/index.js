//Yanni Pang, Sam Kranoff, Abd el Aziz Husssein | 10-23-2020
// Modules
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');

var PORT = 3333;
var HOST = '10.0.1.199';
var remoteport;
var remoteadd;
var data;
var server = dgram.createSocket("udp4"  );

server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);

});
server.on('message', function (message, remote) {
    data = message.toString();
    remoteport = remote.port;
    remoteadd = remote.address;
    console.log(data);
    server.send(data.toString(),remote.port,remote.address,function(error){
        if(error){
            console.log('MEH!');
        }
        else{
            console.log('Sent: Ok!');
        }
    });


});
server.bind(PORT, HOST);

// Iterate from 0 to 10 (then loop) at 1 second intervals

setInterval( function() {
    console.log('Read:', remoteport);    // Log to console
    io.emit('message', data);      // 4. Emit to message event

}, 500);



// 3. Points to index.html to serve webpage
app.get('/', function(req, res){
    res.sendFile(__dirname + '/index.html');
});

// 2. User socket connection
io.on('connection', function(socket){
    console.log('a user connected');
    socket.on('disconnect', function(){
        console.log('user disconnected');
    });
    socket.on('toggle', function (message) {

            server.send("1",remoteport,remoteadd,function(error){
                if(error){
                    console.log('MEH!');
                }
                else{
                    console.log('Sent:', message);
                }
            });

        console.log(data);
    });
});


// 1. Listening on localhost:3000
http.listen(8080, function() {
    console.log('listening on *:3000');
});

