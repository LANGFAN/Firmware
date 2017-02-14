
#include "shoubei_baro.hpp"
#include <cmath>

const char *const UavcanBaroBridge::NAME = "sb_baro";

UavcanBaroBridge::UavcanBaroBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_baro", "/dev/uavcan/baro", BARO_BASE_DEVICE_PATH, ORB_ID(shoubei_sensor_baro)),
	_sub_air_pressure_data(node),
	_reports(2, sizeof(baro_report))
{ }

int UavcanBaroBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_air_pressure_data.start(AirPressureCbBinder(this, &UavcanBaroBridge::air_pressure_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}
	return 0;
}

ssize_t UavcanBaroBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *baro_buf = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	while (count--) {
		if (_reports.get(baro_buf)) {
			ret += sizeof(*baro_buf);
			baro_buf++;
		}
	}

	/* if there was no data, warn the caller */
	return ret ? ret : -EAGAIN;
}

int UavcanBaroBridge::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case BAROIOCSMSLPRESSURE: {
			if ((arg < 80000) || (arg > 120000)) {
				return -EINVAL;

			} else {
				DEVICE_LOG("new msl pressure %u", _msl_pressure);
				_msl_pressure = arg;
				return OK;
			}
		}

	case BAROIOCGMSLPRESSURE: {
			return _msl_pressure;
		}

	case SENSORIOCSPOLLRATE: {
			// not supported yet, pretend that everything is ok
			return OK;
		}

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = irqsave();

			if (!_reports.resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);

			return OK;
		}

	default: {
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

void UavcanBaroBridge::air_pressure_sub_cb(const
		uavcan::ReceivedDataStructure<uavcan::equipment::imu::Baro> &msg)
{
	struct shoubei_sensor_baro_s baro;


	int pressure = msg.baro[0];
	int temperature = msg.baro[1];
	float _baro_mbar_range_scale = 0.04; //scale = 40 ubar/LSB  0.04 mbar/LSB  0.00004 bar/LSB
	float _baro_bar_range_scale = 0.00004;

	baro.timestamp = hrt_absolute_time();

	if(pressure < 32767){
		baro.pressure = (float)msg.baro[0] * _baro_mbar_range_scale;  			//mbar
		pressure = (float)msg.baro[0] * _baro_bar_range_scale;					//bar
	}else{
		baro.pressure = -(float)(~(msg.baro[0])+1) * (_baro_mbar_range_scale);	//mbar
		pressure = -(float)(~(msg.baro[0])+1) * (_baro_bar_range_scale);		//bar
	}

	if(temperature < 10619){
		baro.temperature = (float)msg.baro[1] * 0.00565 + 25.0;
	}else{
		baro.temperature = -(float)(~(msg.baro[1])+1) * (0.00565) + 25.0;

	}

	/*
	 * Altitude computation
	 * Refer to the MS5611 driver for details
	 */
	const double T1 = 15.0 + 273.15; // temperature at base height in Kelvin
	const double a  = -6.5 / 1000;   // temperature gradient in degrees per metre
	const double g  = 9.80665;       // gravity constant in m/s/s
	const double R  = 287.05;        // ideal gas constant in J/kg/K

	const double p1 = _msl_pressure / 1000.0;      // current pressure at MSL in kPa
	const double p = double(pressure) / double(1000.0); // measured pressure in kPa

	baro.altitude = (((std::pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;

	if (_baro_pub == NULL) {
		_baro_pub = orb_advertise(ORB_ID(shoubei_sensor_baro), &baro);
	} else {
		orb_publish(ORB_ID(shoubei_sensor_baro), _baro_pub, &baro);
	}
}
