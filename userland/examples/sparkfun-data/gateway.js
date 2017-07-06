
// Set advertisement rate to be once per second. This must be set before
// including the bleno library (which is included from eddystone-beacon).
process.env['BLENO_ADVERTISING_INTERVAL'] = 1000;


process.env['NOBLE_MULTI_ROLE'] = 1;

var http       = require('http');
var os         = require('os');

var bleno      = require('bleno');
var debug      = require('debug')('http-gateway');
var httpparser = require('http-string-parser');
var noble      = require('noble');
var request    = require('request');


// var req = 'GET https://j2x.us/\n\
// host: j2x.us';

// console.log(req);

// var a = httpparser.parseRequest(req);
// console.log(a);

// if (!a.uri.startsWith('h')) {
// 	a.uri = 'http://' + a.headers.host + a.uri;
// }

// request(a, function (error, response, body) {
// 	console.log('error:', error);
// 	console.log('statusCode:', response && response.statusCode);
// 	console.log('body:', body);
// });

var DEVICE_NAME         = 'http-gateway';
var SERVICE_UUID        = '16ba0001cf44461eb8894f9a90f6b330';
var CHARACTERISTIC_UUID = '16ba0002cf44461eb8894f9a90f6b330';

var WANTSERVER_UUID = '16ba0005cf44461eb8894f9a90f6b330';

var _notify_callback = null;

var _bleno_state = null;
var _bleno_setup_timer = null;


var _body = 'adf';

function setup_services () {
	console.log('setup services')
	bleno.setServices([
		new bleno.PrimaryService({
			uuid: SERVICE_UUID,
			characteristics: [
				new bleno.Characteristic({
					uuid: CHARACTERISTIC_UUID,
					properties: ['read', 'write', 'writeWithoutResponse', 'notify'],
					onReadRequest: function (offset, callback) {
						if (offset == 0) {
							debug('Read characteristic.');
						}

						var body = Buffer.from(_body);
						var ret = Buffer.from('');
						if (offset < body.length) {
							ret = body.slice(offset);
						}

						callback(bleno.Characteristic.RESULT_SUCCESS, ret);
					},
					onWriteRequest: function(data, offset, withoutResponse, callback) {
						console.log('got write')
						console.log(data)

						var msg = data.toString('utf-8');
						console.log(msg);
						var b = httpparser.parseRequest(msg);


						if (!b.uri.startsWith('h')) {
							var host = '';
							if ('Host' in b.headers) {
								host = b.headers.Host;
							} else if ('host' in b.headers) {
								host = b.headers.host;
							} else {
								console.log('no host??');
							}
							b.uri = 'https://' + host + b.uri;
						}
						console.log(b);


						request(b, function (error, response, body) {
							if (error) {
								console.log(error);
							} else if (response.statusCode == 200) {
								console.log('error:', error);
								console.log('statusCode:', response && response.statusCode);
								console.log('headers:', response.headers);
								console.log('body:', body);
								console.log(typeof body);

								_body = body;
								if (_notify_callback) {
									_notify_callback(Buffer.from('ready!'));
								}
							} else {
								console.log('nooo');
							}
						});

						callback(bleno.Characteristic.RESULT_SUCCESS);
					},
					onSubscribe: function (maxValueSize, updateValueCallback) {
						debug('subscribe characteristic');
						_notify_callback = updateValueCallback;
					},
					onUnsubscribe: function () {
						_notify_callback = null;
					}
				})
			]
		})
	], function (error2) {
		if (error2) {
			console.log('error creating services');
			console.log(error2);
		}
	});
}


function start_scanning () {
	noble.startScanning([WANTSERVER_UUID], true);
}




bleno.on('stateChange', function(state) {
	debug('on -> stateChange: ' + state);

	setup_services();
});


noble.on('stateChange', function(state) {
	debug('noble on -> stateChange: ' + state);

	if (state === 'poweredOn') {
		start_scanning();
	} else {
		noble.stopScanning();
	}
});


var _peripheral = null;

noble.on('discover', function (peripheral) {
	console.log(peripheral.address + ' - ' + peripheral.advertisement.serviceUuids);

	if (peripheral.advertisement.serviceUuids.length > 0 &&
		peripheral.advertisement.serviceUuids.indexOf(WANTSERVER_UUID) >= 0) {
		console.log('yay wants us!')

		// Once we have found a client, stop scanning for new ones.
		noble.stopScanning();

		// Save a pointer to this peripheral so we can disconnect from it.
		_peripheral = peripheral;

		// After a while forcefully disconnect and start scanning again.
		_peripheral_timeout = setTimeout(function () {
			_peripheral_timeout = null;
			_peripheral.disconnect();
			start_scanning();
		}, 5000);

		// Connect to let the device use the gateway.
		peripheral.connect(function (err) {
			if (err) {
				console.log('error connecting')
				console.log(err)
			} else {
				console.log('hooray connected to device!')
			}
		});
	}
});



noble.on('scanStart', function () {
	debug('scaninng started');
});
noble.on('scanStop', function () {
	debug('scaninng stopped!');
});

bleno.on('advertisingStart', function(error) {
	debug('on -> advertisingStart: ' + (error ? 'error ' + error : 'success'));
});

bleno.on('advertisingStartError', function (err) {
	debug('adv start err: ' + err);
	debug(err);
});

bleno.on('disconnect', function (client_address) {
	debug('DISCONNECT ' + client_address);
	_notify_callback = null;
	if (_peripheral_timeout) {
		clearTimeout(_peripheral_timeout);
	}
	start_scanning();
});

bleno.on('advertisingStop', function () {
	debug('ADV STOPPED');
});
