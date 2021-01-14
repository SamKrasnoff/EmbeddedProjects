// Modules
var level = require('level');
var express = require('express');
var app = require('express')();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var dgram = require('dgram');
var fs = require('fs'), filename = 'smoke.txt';

// Create or open the underlying LevelDB store
var db = level('./mydb', {valueEncoding: 'json'});


    fs.readFile(filename, 'utf8', function (err, data) {
        if (err) throw err;

        let csvLines = [];
        let points = [];
        csvLines = data.split(/[\r?\n|\r|\n]+/);

        for (let i = 1; i < csvLines.length; i++) {
            if (csvLines[i].length > 0) {
                points = csvLines[i].split("\t");
                let id = parseInt(points[1]);
                let time = parseInt(points[0]);
                let smoke = parseInt(points[2]);
                let temp = parseFloat(points[3]);
                let value = [{id: id, time: time, smoke: smoke, temp: temp}];
                db.put([i], value, function (err) {
                    if (err) return console.log('Ooops!', err) // some kind of I/O error
                });
                var msg = {[i]: value};
                console.log("Added: " + Object.keys(msg));
                io.emit('message', msg);
            }
        }

    });

// Points to index.html to serve webpage
app.get('/', function(req, res){
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
io.on('connection', function(socket){
    console.log('a user connected');
    clientConnected = 0;

    // Call function to stream database data
    readDB();
    clientConnected = 1;
    socket.on('disconnect', function(){
        console.log('user disconnected');
    });
});

// Listening on localhost:3000
http.listen(3000, function() {
    console.log('listening on *:3000');
});


