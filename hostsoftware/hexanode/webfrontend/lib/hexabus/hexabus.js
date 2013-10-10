var exec = require('child_process').exec;
var v6 = require('ipv6').v6;
var os = require('os');
var fs = require('fs');

var hexabus = function() {
	this.rename_device = function(addr, newName, cb) {
		addr = new v6.Address(addr);
		if (!addr.valid) {
			throw "Invalid address";
		}
		var command = "hexaupload --ip " + addr.canonicalForm() + " --rename '" + newName.replace("'", "''") + "'";
		exec(command, function(error, stdout, stderr) {
			if (error && cb) {
				cb(error);
			} else if (cb) {
				cb();
			}
		});
	};

	this.read_current_config = function() {
		var ifaces = os.networkInterfaces();

		var config = {
			addrs: {
				lan: {
					v4: [],
					v6: []
				},

				hxb: {
					v6: []
				}
			}
		};

		for (var dev in ifaces) {
			var target, read_v4;
			switch (dev) {
				case 'eth0':
					target = config.addrs.lan;
					read_v4 = true;
					break;

				case 'usb0':
					target = config.addrs.hxb;
					read_v4 = false;
					break;

				default:
					continue;
			}

			ifaces[dev].forEach(function(detail) {
				if (detail.family == 'IPv6') {
					target.v6.push(detail.address);
				} else if (detail.family == 'IPv4' && read_v4) {
					target.v4.push(detail.address);
				}
			});
		}

		return config;
	};

	this.get_heartbeat_state = function(cb) {
		fs.readFile('/var/run/hexabus_msg_heartbeat.state', { encoding: 'utf8' }, function(err, data) {
			if (err) {
				cb(err, undefined);
			} else {
				var lines = data.split('\n');
				lines = lines.slice(0, -1);
				var messages = lines.slice(0, -1);
				var ret = parseInt(lines[lines.length - 1]);
				cb(undefined, {
					code: ret,
					messages: messages
				});
			}
		});
	};
};

module.exports = hexabus;