//Yanni Pang, Sam Krasnoff, Abdelaziz Hussein
var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs');
var csv = require("csv-parse");


// viewed at http://localhost:8080
app.get('/', function(req, res) {
    res.sendFile(path.join(__dirname + '/index.html'));
});

// request data at http://localhost:8080/data or just "/data"
app.get('/data', function(req, res) {
    var data = [];  // Array to hold all csv data
    fs.createReadStream('data.csv')
        .pipe(csv())
        .on('data', (row) => {
            console.log();
            data.push(row);  // Add row of data to array
        })
        .on('end', () => {
            console.log('CSV file successfully processed');
            res.send(data);  // Send array of data back to requestor
        });
});

//Yanni Pang Sam Krasnoff Abd el Hussain
const SerialPort = require('serialport');
const Delimiter = require('@serialport/parser-delimiter');
const port = new SerialPort("/dev/ttyUSB0", { baudRate: 115200 });
// let d = new Date();
// let time = d.getTime();
const parser = port.pipe(new Delimiter({ delimiter: '\n'}));
fs.writeFile('data.csv', "",function () {

});
parser.on('data', function(data){
    // let time2 = new Date();
    // let seconds = time2.getTime();
    // let time3 = seconds - time;
    // let line = data.toString();
    // let obj = JSON.parse(line);
    // obj.time = parseInt(time3/1000);
    // let obj2 = JSON.stringify(obj);
    // console.log(obj);
    fs.appendFile('data.csv', data, (err) => {
        if (err) throw err;
        console.log('data written');
    });

});


app.listen(8080);
