
// Set advertisement rate to be once per second. This must be set before
// including the bleno library (which is included from eddystone-beacon).
process.env['BLENO_ADVERTISING_INTERVAL'] = 1000;

// Needed to allow us to use both noble and bleno
process.env['NOBLE_MULTI_ROLE'] = 1;

var http       = require('http');
var os         = require('os');

var bleno      = require('bleno');
var debug      = require('debug')('http-gateway');
var httpparser = require('http-string-parser');
var noble      = require('noble');
var request    = require('request');



var DEVICE_NAME       = 'http-gateway';

// Main HTTP over BLE service.
var SERVICE_UUID      = '16ba0001cf44461eb8894f9a90f6b330';
// Characteristic for doing an HTTP request.
var CHAR_HTTP_UUID    = '16ba0002cf44461eb8894f9a90f6b330';
// Characteristic for doing an HTTPS request.
var CHAR_HTTPS_UUID   = '16ba0003cf44461eb8894f9a90f6b330';
// Characteristic for reading the HTTP response body.
var CHAR_BODY_UUID    = '16ba0004cf44461eb8894f9a90f6b330';
// Characteristic for reading the HTTP response headers.
var CHAR_HEADERS_UUID = '16ba0005cf44461eb8894f9a90f6b330';
// UUID that says the device is looking for a HTTP over BLE gateway.
var WANTSERVER_UUID   = '16ba0006cf44461eb8894f9a90f6b330';


var _notify_headers_callback = null;
var _notify_body_callback = null;
var _headers = '';
var _body = '';

function do_request(http_string, mode) {
	var http_request = httpparser.parseRequest(http_string);

	if (!http_request.uri.startsWith(mode + '://')) {
		var host = '';
		if ('Host' in http_request.headers) {
			host = http_request.headers.Host;
		} else if ('host' in http_request.headers) {
			host = http_request.headers.host;
		}
		http_request.uri = mode + '://' + host + http_request.uri;
	}

	request(http_request, function (error, response, body) {
		if (error) {
			debug('Error performing ' + mode + ' request.');
			debug(error);
		} else {
			_headers = response.headers;
			_body = body;
			if (_notify_headers_callback) {
				_notify_headers_callback(Buffer.from('headers'));
			}
			if (_notify_body_callback) {
				_notify_body_callback(Buffer.from('body'));
			}
		}
	});
}

function setup_services () {
	debug('Setting up services.');

	bleno.setServices([
		new bleno.PrimaryService({
			uuid: SERVICE_UUID,
			characteristics: [
				new bleno.Characteristic({
					uuid: CHAR_HTTP_UUID,
					properties: ['write', 'writeWithoutResponse'],
					onWriteRequest: function(data, offset, withoutResponse, callback) {
						console.log('wrote')
						var msg = data.toString('utf-8');
						do_request(msg, 'http');
						callback(bleno.Characteristic.RESULT_SUCCESS);
					}
				}),
				new bleno.Characteristic({
					uuid: CHAR_HTTPS_UUID,
					properties: ['write', 'writeWithoutResponse'],
					onWriteRequest: function(data, offset, withoutResponse, callback) {
						console.log('wrote')
						var msg = data.toString('utf-8');
						do_request(msg, 'https');
						callback(bleno.Characteristic.RESULT_SUCCESS);
					}
				}),
				new bleno.Characteristic({
					uuid: CHAR_BODY_UUID,
					properties: ['read', 'notify'],
					onReadRequest: function (offset, callback) {
						if (offset == 0) {
							debug('Read body characteristic.');
						}

						var body = Buffer.from(_body);
						var ret = Buffer.from('');
						if (offset < body.length) {
							ret = body.slice(offset);
						}

						callback(bleno.Characteristic.RESULT_SUCCESS, ret);
					},
					onSubscribe: function (maxValueSize, updateValueCallback) {
						debug('subscribe characteristic');
						_notify_body_callback = updateValueCallback;
					},
					onUnsubscribe: function () {
						_notify_body_callback = null;
					}
				}),
				new bleno.Characteristic({
					uuid: CHAR_HEADERS_UUID,
					properties: ['read', 'notify'],
					onReadRequest: function (offset, callback) {
						if (offset == 0) {
							debug('Read headers characteristic.');
						}

						var body = Buffer.from(_headers);
						var ret = Buffer.from('');
						if (offset < body.length) {
							ret = body.slice(offset);
						}

						callback(bleno.Characteristic.RESULT_SUCCESS, ret);
					},
					onSubscribe: function (maxValueSize, updateValueCallback) {
						debug('subscribe characteristic');
						_notify_headers_callback = updateValueCallback;
					},
					onUnsubscribe: function () {
						_notify_headers_callback = null;
					}
				})
			]
		})
	], function (error2) {
		if (error2) {
			debug('error creating services');
			debug(error2);
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
