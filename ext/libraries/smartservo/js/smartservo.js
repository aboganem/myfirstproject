// mBot.js

(function(ext) {
    var device = null;
    var _rxBuf = [];

    // Sensor states:
   var values={
	   "Speed":0, 
	   "Temperture":1,
		"Current":2,
		"Voltage":3, 
		"Angle":4
   }
    ext.initAngle = function(devId){
		runPackage(0x40,0x07,0x05,devId);
	};
	ext.resetAngle = function(devId){
		runPackage(0x40,0x08,0x05,devId);
	};
	ext.runServoAbsolute = function(devId,angle,speed){
        runPackage(0x40,0x04,0x05,devId,int2array(angle),float2array(speed));
	};
	ext.runServoRelative = function(devId,angle,speed){
		runPackage(0x40,0x05,0x05,devId,int2array(angle),float2array(speed));
	};
	ext.runServoAsDC = function(devId,speed){
		runPackage(0x40,0x06,0x05,devId,short2array(speed));
	};
	ext.runLED = function(devId,r,g,b){
		runPackage(0x40,0x02,0x05,devId,r,g,b);
	};
	
    ext.readServoProperty = function(nextID,deviceId,property){
    	var value = values[property];
		getPackage(nextID,0x40,value+9,5,deviceId);//0x09 0x0a 0x0b 0x0c 0x0d
    };
	
	function sendPackage(argList, type){
		var bytes = [0xff, 0x55, 0, 0, type];
		for(var i=0;i<argList.length;++i){
			var val = argList[i];
			if(val.constructor == "[class Array]"){
				bytes = bytes.concat(val);
			}else{
				bytes.push(val);
			}
		}
		bytes[2] = bytes.length - 3;
		device.send(bytes);
	}
	
	function runPackage(){
		sendPackage(arguments, 2);
	}
	function getPackage(){
		var nextID = arguments[0];
		Array.prototype.shift.call(arguments);
		sendPackage(arguments, 1);
	}

    var inputArray = [];
	var _isParseStart = false;
	var _isParseStartIndex = 0;
    function processData(bytes) {
		var len = bytes.length;
		if(_rxBuf.length>30){
			_rxBuf = [];
		}
		for(var index=0;index<bytes.length;index++){
			var c = bytes[index];
			_rxBuf.push(c);
			if(_rxBuf.length>=2){
				if(_rxBuf[_rxBuf.length-1]==0x55 && _rxBuf[_rxBuf.length-2]==0xff){
					_isParseStart = true;
					_isParseStartIndex = _rxBuf.length-2;
				}
				if(_rxBuf[_rxBuf.length-1]==0xa && _rxBuf[_rxBuf.length-2]==0xd&&_isParseStart){
					_isParseStart = false;
					
					var position = _isParseStartIndex+2;
					var extId = _rxBuf[position];
					position++;
					var type = _rxBuf[position];
					position++;
					//1 byte 2 float 3 short 4 len+string 5 double
					var value;
					switch(type){
						case 1:{
							value = _rxBuf[position];
							position++;
						}
							break;
						case 2:{
							value = readFloat(_rxBuf,position);
							position+=4;
						}
							break;
						case 3:{
							value = readInt(_rxBuf,position,2);
							position+=2;
						}
							break;
						case 4:{
							var l = _rxBuf[position];
							position++;
							value = readString(_rxBuf,position,l);
						}
							break;
						case 5:{
							value = readDouble(_rxBuf,position);
							position+=4;
						}
							break;
						case 6:
							value = readInt(_rxBuf,position,4);
							position+=4;
							break;
					}
					if(type<=6){
						responseValue(extId,value);
					}else{
						responseValue();
					}
					_rxBuf = [];
				}
			} 
		}
    }
	function readFloat(arr,position){
		var f= [arr[position],arr[position+1],arr[position+2],arr[position+3]];
		return parseFloat(f);
	}
	function readInt(arr,position,count){
		var result = 0;
		for(var i=0; i<count; ++i){
			result |= arr[position+i] << (i << 3);
		}
		return result;
	}
	function readDouble(arr,position){
		return readFloat(arr,position);
	}
	function readString(arr,position,len){
		var value = "";
		for(var ii=0;ii<len;ii++){
			value += String.fromCharCode(_rxBuf[ii+position]);
		}
		return value;
	}
    function appendBuffer( buffer1, buffer2 ) {
        return buffer1.concat( buffer2 );
    }

    // Extension API interactions
    var potentialDevices = [];
    ext._deviceConnected = function(dev) {
        potentialDevices.push(dev);
        if (!device) {
            tryNextDevice();
        }
    }

    function tryNextDevice() {
        // If potentialDevices is empty, device will be undefined.
        // That will get us back here next time a device is connected.
        device = potentialDevices.shift();
        if (device) {
            device.open({ stopBits: 0, bitRate: 115200, ctsFlowControl: 0 }, deviceOpened);
        }
    }

    var watchdog = null;
    function deviceOpened(dev) {
        if (!dev) {
            // Opening the port failed.
            tryNextDevice();
            return;
        }
        device.set_receive_handler('mbot',processData);
    };

    ext._deviceRemoved = function(dev) {
        if(device != dev) return;
        device = null;
    };

    ext._shutdown = function() {
        if(device) device.close();
        device = null;
    };

    ext._getStatus = function() {
        if(!device) return {status: 1, msg: 'mBot disconnected'};
        if(watchdog) return {status: 1, msg: 'Probing for mBot'};
        return {status: 2, msg: 'mBot connected'};
    }
    var descriptor = {};
	ScratchExtensions.register('smartservo', descriptor, ext, {type: 'serial'});
})({});
