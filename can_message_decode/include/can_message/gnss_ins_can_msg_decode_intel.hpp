#ifndef _GNSS_INS_CAN_MESSAGE_DECODE_HPP_
#define _GNSS_INS_CAN_MESSAGE_DECODE_HPP_


#include "can_message/can_message.hpp"
#include "can_message/can_message.hpp"
#include "can_message/signal.hpp"

#include "utils/utils.hpp"

/* Frame ids. */
#define LATITUDE_FRAME_ID (0x32eu)
#define LONGITUDE_FRAME_ID (0x32du)
#define ANGRATEVEHICLE_FRAME_ID (0x32cu)
#define HEADINGPITCHROLLSIGMA_FRAME_ID (0x32bu)
#define HEADINGPITCHROLL_FRAME_ID (0x32au)
#define ACCEL_VEHICLE_FRAME_ID (0x329u)
#define VELOCITYLEVELSIGMA_FRAME_ID (0x328u)
#define TIME_FRAME_ID (0x320u)
#define VELOCITYLEVEL_FRAME_ID (0x327u)
#define POSSIGMA_FRAME_ID (0x326u)
#define ALTITUDE_FRAME_ID (0x325u)
#define LATITUDELONGITUDE_FRAME_ID (0x324u)
#define INSSTATUS_FRAME_ID (0x323u)
#define ACCEL_IMU_RAW_FRAME_ID (0x322u)
#define ANG_RATE_RAW_IMU_FRAME_ID (0x321u)


/************ Message Class Defination **********/ 

namespace gnss_ins{

class Latitude : public CanMessage{
public: 
	// 构造函数
	Latitude(){
		this->frame_id = LATITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {""};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Latitude(const _data_type& message_data){
		this->frame_id = LATITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {""};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Latitude(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {""};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Latitude(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {""};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class PosLat2 : public Signal<int64_t , double>{
	public: 
		// 构造函数
		PosLat2(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 64;
			this->is_intel_ = true;
			this->scale_ = 1e-08;
			this->offset_ = 0;
			this->range_min_ = -90;
			this->range_max_ = 90;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. PosLat2
		{
			_data_bin_type poslat2_data_bin(this->poslat2.data);
			for(uint8_t i = 0 ; i < this->poslat2.signal_length() ; i++){
				this->data_bin[i + this->poslat2.start_bit()] = poslat2_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. PosLat2
		{
			_data_bin_type poslat2_data_bin;
			for(uint8_t i = 0; i < this->poslat2.signal_length(); i++)
				poslat2_data_bin[i] = this->data_bin[this->poslat2.start_bit() + i];
			this->poslat2.data = utils::bin2int(poslat2_data_bin, this->poslat2.signal_length());
		}
	}

public: 
	PosLat2 poslat2;
};


class Longitude : public CanMessage{
public: 
	// 构造函数
	Longitude(){
		this->frame_id = LONGITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {""};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Longitude(const _data_type& message_data){
		this->frame_id = LONGITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {""};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Longitude(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {""};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Longitude(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {""};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class PosLon2 : public Signal<int64_t , double>{
	public: 
		// 构造函数
		PosLon2(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 64;
			this->is_intel_ = true;
			this->scale_ = 1e-08;
			this->offset_ = 0;
			this->range_min_ = -180;
			this->range_max_ = 180;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. PosLon2
		{
			_data_bin_type poslon2_data_bin(this->poslon2.data);
			for(uint8_t i = 0 ; i < this->poslon2.signal_length() ; i++){
				this->data_bin[i + this->poslon2.start_bit()] = poslon2_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. PosLon2
		{
			_data_bin_type poslon2_data_bin;
			for(uint8_t i = 0; i < this->poslon2.signal_length(); i++)
				poslon2_data_bin[i] = this->data_bin[this->poslon2.start_bit() + i];
			this->poslon2.data = utils::bin2int(poslon2_data_bin, this->poslon2.signal_length());
		}
	}

public: 
	PosLon2 poslon2;
};


class AngRateVehicle : public CanMessage{
public: 
	// 构造函数
	AngRateVehicle(){
		this->frame_id = ANGRATEVEHICLE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit AngRateVehicle(const _data_type& message_data){
		this->frame_id = ANGRATEVEHICLE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit AngRateVehicle(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit AngRateVehicle(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AngRateX : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateX(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

	class AngRateY : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateY(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

	class AngRateZ : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateZ(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AngRateX
		{
			_data_bin_type angratex_data_bin(this->angratex.data);
			for(uint8_t i = 0 ; i < this->angratex.signal_length() ; i++){
				this->data_bin[i + this->angratex.start_bit()] = angratex_data_bin[i];
			}
		}
		// 2. AngRateY
		{
			_data_bin_type angratey_data_bin(this->angratey.data);
			for(uint8_t i = 0 ; i < this->angratey.signal_length() ; i++){
				this->data_bin[i + this->angratey.start_bit()] = angratey_data_bin[i];
			}
		}
		// 3. AngRateZ
		{
			_data_bin_type angratez_data_bin(this->angratez.data);
			for(uint8_t i = 0 ; i < this->angratez.signal_length() ; i++){
				this->data_bin[i + this->angratez.start_bit()] = angratez_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AngRateX
		{
			_data_bin_type angratex_data_bin;
			for(uint8_t i = 0; i < this->angratex.signal_length(); i++)
				angratex_data_bin[i] = this->data_bin[this->angratex.start_bit() + i];
			this->angratex.data = utils::bin2int(angratex_data_bin, this->angratex.signal_length());
		}
		// 2. AngRateY
		{
			_data_bin_type angratey_data_bin;
			for(uint8_t i = 0; i < this->angratey.signal_length(); i++)
				angratey_data_bin[i] = this->data_bin[this->angratey.start_bit() + i];
			this->angratey.data = utils::bin2int(angratey_data_bin, this->angratey.signal_length());
		}
		// 3. AngRateZ
		{
			_data_bin_type angratez_data_bin;
			for(uint8_t i = 0; i < this->angratez.signal_length(); i++)
				angratez_data_bin[i] = this->data_bin[this->angratez.start_bit() + i];
			this->angratez.data = utils::bin2int(angratez_data_bin, this->angratez.signal_length());
		}
	}

public: 
	AngRateX angratex;
	AngRateY angratey;
	AngRateZ angratez;
};


class HeadingPitchRollSigma : public CanMessage{
public: 
	// 构造函数
	HeadingPitchRollSigma(){
		this->frame_id = HEADINGPITCHROLLSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRollSigma(const _data_type& message_data){
		this->frame_id = HEADINGPITCHROLLSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRollSigma(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRollSigma(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AngleHeadingSigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		AngleHeadingSigma(){dingsigma
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

	class AnglePitchSigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		AnglePitchSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

	class AngleRollSigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		AngleRollSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AngleHeadingSigma
		{
			_data_bin_type angleheadingsigma_data_bin(this->angleheadingsigma.data);
			for(uint8_t i = 0 ; i < this->angleheadingsigma.signal_length() ; i++){
				this->data_bin[i + this->angleheadingsigma.start_bit()] = angleheadingsigma_data_bin[i];
			}
		}
		// 2. AnglePitchSigma
		{
			_data_bin_type anglepitchsigma_data_bin(this->anglepitchsigma.data);
			for(uint8_t i = 0 ; i < this->anglepitchsigma.signal_length() ; i++){
				this->data_bin[i + this->anglepitchsigma.start_bit()] = anglepitchsigma_data_bin[i];
			}
		}
		// 3. AngleRollSigma
		{
			_data_bin_type anglerollsigma_data_bin(this->anglerollsigma.data);
			for(uint8_t i = 0 ; i < this->anglerollsigma.signal_length() ; i++){
				this->data_bin[i + this->anglerollsigma.start_bit()] = anglerollsigma_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AngleHeadingSigma
		{
			_data_bin_type angleheadingsigma_data_bin;
			for(uint8_t i = 0; i < this->angleheadingsigma.signal_length(); i++)
				angleheadingsigma_data_bin[i] = this->data_bin[this->angleheadingsigma.start_bit() + i];
			this->angleheadingsigma.data = utils::bin2int(angleheadingsigma_data_bin, this->angleheadingsigma.signal_length());
		}
		// 2. AnglePitchSigma
		{
			_data_bin_type anglepitchsigma_data_bin;
			for(uint8_t i = 0; i < this->anglepitchsigma.signal_length(); i++)
				anglepitchsigma_data_bin[i] = this->data_bin[this->anglepitchsigma.start_bit() + i];
			this->anglepitchsigma.data = utils::bin2int(anglepitchsigma_data_bin, this->anglepitchsigma.signal_length());
		}
		// 3. AngleRollSigma
		{
			_data_bin_type anglerollsigma_data_bin;
			for(uint8_t i = 0; i < this->anglerollsigma.signal_length(); i++)
				anglerollsigma_data_bin[i] = this->data_bin[this->anglerollsigma.start_bit() + i];
			this->anglerollsigma.data = utils::bin2int(anglerollsigma_data_bin, this->anglerollsigma.signal_length());
		}
	}

public: 
	AngleHeadingSigma angleheadingsigma;
	AnglePitchSigma anglepitchsigma;
	AngleRollSigma anglerollsigma;
};


class HeadingPitchRoll : public CanMessage{
public: 
	// 构造函数
	HeadingPitchRoll(){
		this->frame_id = HEADINGPITCHROLL_FRAME_ID;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRoll(const _data_type& message_data){
		this->frame_id = HEADINGPITCHROLL_FRAME_ID;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRoll(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit HeadingPitchRoll(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AngleHeading : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		AngleHeading(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 360;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

	class AnglePitch : public Signal<int16_t , double>{
	public: 
		// 构造函数
		AnglePitch(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 16;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -90;
			this->range_max_ = 90;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

	class AngleRoll : public Signal<int16_t , double>{
	public: 
		// 构造函数
		AngleRoll(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 32;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -180;
			this->range_max_ = 180;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AngleHeading
		{
			_data_bin_type angleheading_data_bin(this->angleheading.data);
			for(uint8_t i = 0 ; i < this->angleheading.signal_length() ; i++){
				this->data_bin[i + this->angleheading.start_bit()] = angleheading_data_bin[i];
			}
		}
		// 2. AnglePitch
		{
			_data_bin_type anglepitch_data_bin(this->anglepitch.data);
			for(uint8_t i = 0 ; i < this->anglepitch.signal_length() ; i++){
				this->data_bin[i + this->anglepitch.start_bit()] = anglepitch_data_bin[i];
			}
		}
		// 3. AngleRoll
		{
			_data_bin_type angleroll_data_bin(this->angleroll.data);
			for(uint8_t i = 0 ; i < this->angleroll.signal_length() ; i++){
				this->data_bin[i + this->angleroll.start_bit()] = angleroll_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AngleHeading
		{
			_data_bin_type angleheading_data_bin;
			for(uint8_t i = 0; i < this->angleheading.signal_length(); i++)
				angleheading_data_bin[i] = this->data_bin[this->angleheading.start_bit() + i];
			this->angleheading.data = utils::bin2int(angleheading_data_bin, this->angleheading.signal_length());
		}
		// 2. AnglePitch
		{
			_data_bin_type anglepitch_data_bin;
			for(uint8_t i = 0; i < this->anglepitch.signal_length(); i++)
				anglepitch_data_bin[i] = this->data_bin[this->anglepitch.start_bit() + i];
			this->anglepitch.data = utils::bin2int(anglepitch_data_bin, this->anglepitch.signal_length());
		}
		// 3. AngleRoll
		{
			_data_bin_type angleroll_data_bin;
			for(uint8_t i = 0; i < this->angleroll.signal_length(); i++)
				angleroll_data_bin[i] = this->data_bin[this->angleroll.start_bit() + i];
			this->angleroll.data = utils::bin2int(angleroll_data_bin, this->angleroll.signal_length());
		}
	}

public: 
	AngleHeading angleheading;
	AnglePitch anglepitch;
	AngleRoll angleroll;
};


class Accel_Vehicle : public CanMessage{
public: 
	// 构造函数
	Accel_Vehicle(){
		this->frame_id = ACCEL_VEHICLE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_Vehicle(const _data_type& message_data){
		this->frame_id = ACCEL_VEHICLE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_Vehicle(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_Vehicle(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AccelX : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelX(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -8;
			this->range_max_ = 8;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

	class AccelY : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelY(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -8;
			this->range_max_ = 8;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

	class AccelZ : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelZ(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -8;
			this->range_max_ = 8;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AccelX
		{
			_data_bin_type accelx_data_bin(this->accelx.data);
			for(uint8_t i = 0 ; i < this->accelx.signal_length() ; i++){
				this->data_bin[i + this->accelx.start_bit()] = accelx_data_bin[i];
			}
		}
		// 2. AccelY
		{
			_data_bin_type accely_data_bin(this->accely.data);
			for(uint8_t i = 0 ; i < this->accely.signal_length() ; i++){
				this->data_bin[i + this->accely.start_bit()] = accely_data_bin[i];
			}
		}
		// 3. AccelZ
		{
			_data_bin_type accelz_data_bin(this->accelz.data);
			for(uint8_t i = 0 ; i < this->accelz.signal_length() ; i++){
				this->data_bin[i + this->accelz.start_bit()] = accelz_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AccelX
		{
			_data_bin_type accelx_data_bin;
			for(uint8_t i = 0; i < this->accelx.signal_length(); i++)
				accelx_data_bin[i] = this->data_bin[this->accelx.start_bit() + i];
			this->accelx.data = utils::bin2int(accelx_data_bin, this->accelx.signal_length());
		}
		// 2. AccelY
		{
			_data_bin_type accely_data_bin;
			for(uint8_t i = 0; i < this->accely.signal_length(); i++)
				accely_data_bin[i] = this->data_bin[this->accely.start_bit() + i];
			this->accely.data = utils::bin2int(accely_data_bin, this->accely.signal_length());
		}
		// 3. AccelZ
		{
			_data_bin_type accelz_data_bin;
			for(uint8_t i = 0; i < this->accelz.signal_length(); i++)
				accelz_data_bin[i] = this->data_bin[this->accelz.start_bit() + i];
			this->accelz.data = utils::bin2int(accelz_data_bin, this->accelz.signal_length());
		}
	}

public: 
	AccelX accelx;
	AccelY accely;
	AccelZ accelz;
};


class VelocityLevelSigma : public CanMessage{
public: 
	// 构造函数
	VelocityLevelSigma(){
		this->frame_id = VELOCITYLEVELSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevelSigma(const _data_type& message_data){
		this->frame_id = VELOCITYLEVELSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevelSigma(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevelSigma(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class VelESigma : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		VelESigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 65.535;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class VelNSigma : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		VelNSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 16;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 65.535;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class VelUSigma : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		VelUSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 32;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 65.535;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class VelSigma : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		VelSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 48;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 65.535;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. VelESigma
		{
			_data_bin_type velesigma_data_bin(this->velesigma.data);
			for(uint8_t i = 0 ; i < this->velesigma.signal_length() ; i++){
				this->data_bin[i + this->velesigma.start_bit()] = velesigma_data_bin[i];
			}
		}
		// 2. VelNSigma
		{
			_data_bin_type velnsigma_data_bin(this->velnsigma.data);
			for(uint8_t i = 0 ; i < this->velnsigma.signal_length() ; i++){
				this->data_bin[i + this->velnsigma.start_bit()] = velnsigma_data_bin[i];
			}
		}
		// 3. VelUSigma
		{
			_data_bin_type velusigma_data_bin(this->velusigma.data);
			for(uint8_t i = 0 ; i < this->velusigma.signal_length() ; i++){
				this->data_bin[i + this->velusigma.start_bit()] = velusigma_data_bin[i];
			}
		}
		// 4. VelSigma
		{
			_data_bin_type velsigma_data_bin(this->velsigma.data);
			for(uint8_t i = 0 ; i < this->velsigma.signal_length() ; i++){
				this->data_bin[i + this->velsigma.start_bit()] = velsigma_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. VelESigma
		{
			_data_bin_type velesigma_data_bin;
			for(uint8_t i = 0; i < this->velesigma.signal_length(); i++)
				velesigma_data_bin[i] = this->data_bin[this->velesigma.start_bit() + i];
			this->velesigma.data = utils::bin2int(velesigma_data_bin, this->velesigma.signal_length());
		}
		// 2. VelNSigma
		{
			_data_bin_type velnsigma_data_bin;
			for(uint8_t i = 0; i < this->velnsigma.signal_length(); i++)
				velnsigma_data_bin[i] = this->data_bin[this->velnsigma.start_bit() + i];
			this->velnsigma.data = utils::bin2int(velnsigma_data_bin, this->velnsigma.signal_length());
		}
		// 3. VelUSigma
		{
			_data_bin_type velusigma_data_bin;
			for(uint8_t i = 0; i < this->velusigma.signal_length(); i++)
				velusigma_data_bin[i] = this->data_bin[this->velusigma.start_bit() + i];
			this->velusigma.data = utils::bin2int(velusigma_data_bin, this->velusigma.signal_length());
		}
		// 4. VelSigma
		{
			_data_bin_type velsigma_data_bin;
			for(uint8_t i = 0; i < this->velsigma.signal_length(); i++)
				velsigma_data_bin[i] = this->data_bin[this->velsigma.start_bit() + i];
			this->velsigma.data = utils::bin2int(velsigma_data_bin, this->velsigma.signal_length());
		}
	}

public: 
	VelESigma velesigma;
	VelNSigma velnsigma;
	VelUSigma velusigma;
	VelSigma velsigma;
};


class Time : public CanMessage{
public: 
	// 构造函数
	Time(){
		this->frame_id = TIME_FRAME_ID;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Time(const _data_type& message_data){
		this->frame_id = TIME_FRAME_ID;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Time(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Time(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 48;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class GpsWeek : public Signal<uint16_t , int64_t>{
	public: 
		// 构造函数
		GpsWeek(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 65535;
			this->unit_ = "w";
			this->receiver_ = {};
		}
	};

	class GpsTime : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		GpsTime(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 16;
			this->signal_length_ = 32;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 4294967.295;
			this->unit_ = "s";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. GpsWeek
		{
			_data_bin_type gpsweek_data_bin(this->gpsweek.data);
			for(uint8_t i = 0 ; i < this->gpsweek.signal_length() ; i++){
				this->data_bin[i + this->gpsweek.start_bit()] = gpsweek_data_bin[i];
			}
		}
		// 2. GpsTime
		{
			_data_bin_type gpstime_data_bin(this->gpstime.data);
			for(uint8_t i = 0 ; i < this->gpstime.signal_length() ; i++){
				this->data_bin[i + this->gpstime.start_bit()] = gpstime_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. GpsWeek
		{
			_data_bin_type gpsweek_data_bin;
			for(uint8_t i = 0; i < this->gpsweek.signal_length(); i++)
				gpsweek_data_bin[i] = this->data_bin[this->gpsweek.start_bit() + i];
			this->gpsweek.data = utils::bin2int(gpsweek_data_bin, this->gpsweek.signal_length());
		}
		// 2. GpsTime
		{
			_data_bin_type gpstime_data_bin;
			for(uint8_t i = 0; i < this->gpstime.signal_length(); i++)
				gpstime_data_bin[i] = this->data_bin[this->gpstime.start_bit() + i];
			this->gpstime.data = utils::bin2int(gpstime_data_bin, this->gpstime.signal_length());
		}
	}

public: 
	GpsWeek gpsweek;
	GpsTime gpstime;
};


class VelocityLevel : public CanMessage{
public: 
	// 构造函数
	VelocityLevel(){
		this->frame_id = VELOCITYLEVEL_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevel(const _data_type& message_data){
		this->frame_id = VELOCITYLEVEL_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevel(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit VelocityLevel(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class VelE : public Signal<int16_t , double>{
	public: 
		// 构造函数
		VelE(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -327.68;
			this->range_max_ = 327.67;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class VelN : public Signal<int16_t , double>{
	public: 
		// 构造函数
		VelN(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 16;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -327.68;
			this->range_max_ = 327.67;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class VelU : public Signal<int16_t , double>{
	public: 
		// 构造函数
		VelU(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 32;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -327.68;
			this->range_max_ = 327.67;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

	class Vel : public Signal<int16_t , double>{
	public: 
		// 构造函数
		Vel(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 48;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -327.68;
			this->range_max_ = 327.67;
			this->unit_ = "m/s";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. VelE
		{
			_data_bin_type vele_data_bin(this->vele.data);
			for(uint8_t i = 0 ; i < this->vele.signal_length() ; i++){
				this->data_bin[i + this->vele.start_bit()] = vele_data_bin[i];
			}
		}
		// 2. VelN
		{
			_data_bin_type veln_data_bin(this->veln.data);
			for(uint8_t i = 0 ; i < this->veln.signal_length() ; i++){
				this->data_bin[i + this->veln.start_bit()] = veln_data_bin[i];
			}
		}
		// 3. VelU
		{
			_data_bin_type velu_data_bin(this->velu.data);
			for(uint8_t i = 0 ; i < this->velu.signal_length() ; i++){
				this->data_bin[i + this->velu.start_bit()] = velu_data_bin[i];
			}
		}
		// 4. Vel
		{
			_data_bin_type vel_data_bin(this->vel.data);
			for(uint8_t i = 0 ; i < this->vel.signal_length() ; i++){
				this->data_bin[i + this->vel.start_bit()] = vel_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. VelE
		{
			_data_bin_type vele_data_bin;
			for(uint8_t i = 0; i < this->vele.signal_length(); i++)
				vele_data_bin[i] = this->data_bin[this->vele.start_bit() + i];
			this->vele.data = utils::bin2int(vele_data_bin, this->vele.signal_length());
		}
		// 2. VelN
		{
			_data_bin_type veln_data_bin;
			for(uint8_t i = 0; i < this->veln.signal_length(); i++)
				veln_data_bin[i] = this->data_bin[this->veln.start_bit() + i];
			this->veln.data = utils::bin2int(veln_data_bin, this->veln.signal_length());
		}
		// 3. VelU
		{
			_data_bin_type velu_data_bin;
			for(uint8_t i = 0; i < this->velu.signal_length(); i++)
				velu_data_bin[i] = this->data_bin[this->velu.start_bit() + i];
			this->velu.data = utils::bin2int(velu_data_bin, this->velu.signal_length());
		}
		// 4. Vel
		{
			_data_bin_type vel_data_bin;
			for(uint8_t i = 0; i < this->vel.signal_length(); i++)
				vel_data_bin[i] = this->data_bin[this->vel.start_bit() + i];
			this->vel.data = utils::bin2int(vel_data_bin, this->vel.signal_length());
		}
	}

public: 
	VelE vele;
	VelN veln;
	VelU velu;
	Vel vel;
};


class PosSigma : public CanMessage{
public: 
	// 构造函数
	PosSigma(){
		this->frame_id = POSSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit PosSigma(const _data_type& message_data){
		this->frame_id = POSSIGMA_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit PosSigma(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit PosSigma(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class PosESigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		PosESigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "m";
			this->receiver_ = {};
		}
	};

	class PosNsigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		PosNsigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "m";
			this->receiver_ = {};
		}
	};

	class PosUsigma : public Signal<uint32_t , double>{
	public: 
		// 构造函数
		PosUsigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 100;
			this->unit_ = "m";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. PosESigma
		{
			_data_bin_type posesigma_data_bin(this->posesigma.data);
			for(uint8_t i = 0 ; i < this->posesigma.signal_length() ; i++){
				this->data_bin[i + this->posesigma.start_bit()] = posesigma_data_bin[i];
			}
		}
		// 2. PosNsigma
		{
			_data_bin_type posnsigma_data_bin(this->posnsigma.data);
			for(uint8_t i = 0 ; i < this->posnsigma.signal_length() ; i++){
				this->data_bin[i + this->posnsigma.start_bit()] = posnsigma_data_bin[i];
			}
		}
		// 3. PosUsigma
		{
			_data_bin_type posusigma_data_bin(this->posusigma.data);
			for(uint8_t i = 0 ; i < this->posusigma.signal_length() ; i++){
				this->data_bin[i + this->posusigma.start_bit()] = posusigma_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. PosESigma
		{
			_data_bin_type posesigma_data_bin;
			for(uint8_t i = 0; i < this->posesigma.signal_length(); i++)
				posesigma_data_bin[i] = this->data_bin[this->posesigma.start_bit() + i];
			this->posesigma.data = utils::bin2int(posesigma_data_bin, this->posesigma.signal_length());
		}
		// 2. PosNsigma
		{
			_data_bin_type posnsigma_data_bin;
			for(uint8_t i = 0; i < this->posnsigma.signal_length(); i++)
				posnsigma_data_bin[i] = this->data_bin[this->posnsigma.start_bit() + i];
			this->posnsigma.data = utils::bin2int(posnsigma_data_bin, this->posnsigma.signal_length());
		}
		// 3. PosUsigma
		{
			_data_bin_type posusigma_data_bin;
			for(uint8_t i = 0; i < this->posusigma.signal_length(); i++)
				posusigma_data_bin[i] = this->data_bin[this->posusigma.start_bit() + i];
			this->posusigma.data = utils::bin2int(posusigma_data_bin, this->posusigma.signal_length());
		}
	}

public: 
	PosESigma posesigma;
	PosNsigma posnsigma;
	PosUsigma posusigma;
};


class Altitude : public CanMessage{
public: 
	// 构造函数
	Altitude(){
		this->frame_id = ALTITUDE_FRAME_ID;
		this->message_length = 32;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Altitude(const _data_type& message_data){
		this->frame_id = ALTITUDE_FRAME_ID;
		this->message_length = 32;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Altitude(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 32;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Altitude(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 32;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class PosAlt : public Signal<int32_t , double>{
	public: 
		// 构造函数
		PosAlt(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 32;
			this->is_intel_ = true;
			this->scale_ = 0.001;
			this->offset_ = 0;
			this->range_min_ = -2147483.648;
			this->range_max_ = 2147483.648;
			this->unit_ = "m";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. PosAlt
		{
			_data_bin_type posalt_data_bin(this->posalt.data);
			for(uint8_t i = 0 ; i < this->posalt.signal_length() ; i++){
				this->data_bin[i + this->posalt.start_bit()] = posalt_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. PosAlt
		{
			_data_bin_type posalt_data_bin;
			for(uint8_t i = 0; i < this->posalt.signal_length(); i++)
				posalt_data_bin[i] = this->data_bin[this->posalt.start_bit() + i];
			this->posalt.data = utils::bin2int(posalt_data_bin, this->posalt.signal_length());
		}
	}

public: 
	PosAlt posalt;
};


class LatitudeLongitude : public CanMessage{
public: 
	// 构造函数
	LatitudeLongitude(){
		this->frame_id = LATITUDELONGITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit LatitudeLongitude(const _data_type& message_data){
		this->frame_id = LATITUDELONGITUDE_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit LatitudeLongitude(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit LatitudeLongitude(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class PosLat : public Signal<int32_t , double>{
	public: 
		// 构造函数
		PosLat(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 32;
			this->is_intel_ = true;
			this->scale_ = 1e-07;
			this->offset_ = 0;
			this->range_min_ = -214.7483648;
			this->range_max_ = 214.7483648;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

	class PosLon : public Signal<int32_t , double>{
	public: 
		// 构造函数
		PosLon(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 32;
			this->signal_length_ = 32;
			this->is_intel_ = true;
			this->scale_ = 1e-07;
			this->offset_ = 0;
			this->range_min_ = -214.7483648;
			this->range_max_ = 214.7483648;
			this->unit_ = "deg";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. PosLat
		{
			_data_bin_type poslat_data_bin(this->poslat.data);
			for(uint8_t i = 0 ; i < this->poslat.signal_length() ; i++){
				this->data_bin[i + this->poslat.start_bit()] = poslat_data_bin[i];
			}
		}
		// 2. PosLon
		{
			_data_bin_type poslon_data_bin(this->poslon.data);
			for(uint8_t i = 0 ; i < this->poslon.signal_length() ; i++){
				this->data_bin[i + this->poslon.start_bit()] = poslon_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. PosLat
		{
			_data_bin_type poslat_data_bin;
			for(uint8_t i = 0; i < this->poslat.signal_length(); i++)
				poslat_data_bin[i] = this->data_bin[this->poslat.start_bit() + i];
			this->poslat.data = utils::bin2int(poslat_data_bin, this->poslat.signal_length());
		}
		// 2. PosLon
		{
			_data_bin_type poslon_data_bin;
			for(uint8_t i = 0; i < this->poslon.signal_length(); i++)
				poslon_data_bin[i] = this->data_bin[this->poslon.start_bit() + i];
			this->poslon.data = utils::bin2int(poslon_data_bin, this->poslon.signal_length());
		}
	}

public: 
	PosLat poslat;
	PosLon poslon;
};


class InsStatus : public CanMessage{
public: 
	// 构造函数
	InsStatus(){
		this->frame_id = INSSTATUS_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit InsStatus(const _data_type& message_data){
		this->frame_id = INSSTATUS_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit InsStatus(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit InsStatus(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class system_state : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		system_state(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 0;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 9;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class GpsNumSatsUsed : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		GpsNumSatsUsed(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 8;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 255;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class satellite_status : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		satellite_status(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 16;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 9;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class GpsNumSats2Used : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		GpsNumSats2Used(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 24;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 255;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class GpsAge : public Signal<uint16_t , double>{
	public: 
		// 构造函数
		GpsAge(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 32;
			this->signal_length_ = 16;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 655.35;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class GpsNumSats : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		GpsNumSats(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 48;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 255;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

	class GpsNumSats2 : public Signal<uint8_t , int64_t>{
	public: 
		// 构造函数
		GpsNumSats2(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 56;
			this->signal_length_ = 8;
			this->is_intel_ = true;
			this->scale_ = 1;
			this->offset_ = 0;
			this->range_min_ = 0;
			this->range_max_ = 255;
			this->unit_ = "";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. system_state
		{
			_data_bin_type system_state_data_bin(this->system_state.data);
			for(uint8_t i = 0 ; i < this->system_state.signal_length() ; i++){
				this->data_bin[i + this->system_state.start_bit()] = system_state_data_bin[i];
			}
		}
		// 2. GpsNumSatsUsed
		{
			_data_bin_type gpsnumsatsused_data_bin(this->gpsnumsatsused.data);
			for(uint8_t i = 0 ; i < this->gpsnumsatsused.signal_length() ; i++){
				this->data_bin[i + this->gpsnumsatsused.start_bit()] = gpsnumsatsused_data_bin[i];
			}
		}
		// 3. satellite_status
		{
			_data_bin_type satellite_status_data_bin(this->satellite_status.data);
			for(uint8_t i = 0 ; i < this->satellite_status.signal_length() ; i++){
				this->data_bin[i + this->satellite_status.start_bit()] = satellite_status_data_bin[i];
			}
		}
		// 4. GpsNumSats2Used
		{
			_data_bin_type gpsnumsats2used_data_bin(this->gpsnumsats2used.data);
			for(uint8_t i = 0 ; i < this->gpsnumsats2used.signal_length() ; i++){
				this->data_bin[i + this->gpsnumsats2used.start_bit()] = gpsnumsats2used_data_bin[i];
			}
		}
		// 5. GpsAge
		{
			_data_bin_type gpsage_data_bin(this->gpsage.data);
			for(uint8_t i = 0 ; i < this->gpsage.signal_length() ; i++){
				this->data_bin[i + this->gpsage.start_bit()] = gpsage_data_bin[i];
			}
		}
		// 6. GpsNumSats
		{
			_data_bin_type gpsnumsats_data_bin(this->gpsnumsats.data);
			for(uint8_t i = 0 ; i < this->gpsnumsats.signal_length() ; i++){
				this->data_bin[i + this->gpsnumsats.start_bit()] = gpsnumsats_data_bin[i];
			}
		}
		// 7. GpsNumSats2
		{
			_data_bin_type gpsnumsats2_data_bin(this->gpsnumsats2.data);
			for(uint8_t i = 0 ; i < this->gpsnumsats2.signal_length() ; i++){
				this->data_bin[i + this->gpsnumsats2.start_bit()] = gpsnumsats2_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. system_state
		{
			_data_bin_type system_state_data_bin;
			for(uint8_t i = 0; i < this->system_state.signal_length(); i++)
				system_state_data_bin[i] = this->data_bin[this->system_state.start_bit() + i];
			this->system_state.data = utils::bin2int(system_state_data_bin, this->system_state.signal_length());
		}
		// 2. GpsNumSatsUsed
		{
			_data_bin_type gpsnumsatsused_data_bin;
			for(uint8_t i = 0; i < this->gpsnumsatsused.signal_length(); i++)
				gpsnumsatsused_data_bin[i] = this->data_bin[this->gpsnumsatsused.start_bit() + i];
			this->gpsnumsatsused.data = utils::bin2int(gpsnumsatsused_data_bin, this->gpsnumsatsused.signal_length());
		}
		// 3. satellite_status
		{
			_data_bin_type satellite_status_data_bin;
			for(uint8_t i = 0; i < this->satellite_status.signal_length(); i++)
				satellite_status_data_bin[i] = this->data_bin[this->satellite_status.start_bit() + i];
			this->satellite_status.data = utils::bin2int(satellite_status_data_bin, this->satellite_status.signal_length());
		}
		// 4. GpsNumSats2Used
		{
			_data_bin_type gpsnumsats2used_data_bin;
			for(uint8_t i = 0; i < this->gpsnumsats2used.signal_length(); i++)
				gpsnumsats2used_data_bin[i] = this->data_bin[this->gpsnumsats2used.start_bit() + i];
			this->gpsnumsats2used.data = utils::bin2int(gpsnumsats2used_data_bin, this->gpsnumsats2used.signal_length());
		}
		// 5. GpsAge
		{
			_data_bin_type gpsage_data_bin;
			for(uint8_t i = 0; i < this->gpsage.signal_length(); i++)
				gpsage_data_bin[i] = this->data_bin[this->gpsage.start_bit() + i];
			this->gpsage.data = utils::bin2int(gpsage_data_bin, this->gpsage.signal_length());
		}
		// 6. GpsNumSats
		{
			_data_bin_type gpsnumsats_data_bin;
			for(uint8_t i = 0; i < this->gpsnumsats.signal_length(); i++)
				gpsnumsats_data_bin[i] = this->data_bin[this->gpsnumsats.start_bit() + i];
			this->gpsnumsats.data = utils::bin2int(gpsnumsats_data_bin, this->gpsnumsats.signal_length());
		}
		// 7. GpsNumSats2
		{
			_data_bin_type gpsnumsats2_data_bin;
			for(uint8_t i = 0; i < this->gpsnumsats2.signal_length(); i++)
				gpsnumsats2_data_bin[i] = this->data_bin[this->gpsnumsats2.start_bit() + i];
			this->gpsnumsats2.data = utils::bin2int(gpsnumsats2_data_bin, this->gpsnumsats2.signal_length());
		}
	}

public: 
	system_state system_state;
	GpsNumSatsUsed gpsnumsatsused;
	satellite_status satellite_status;
	GpsNumSats2Used gpsnumsats2used;
	GpsAge gpsage;
	GpsNumSats gpsnumsats;
	GpsNumSats2 gpsnumsats2;
};


class Accel_IMU_Raw : public CanMessage{
public: 
	// 构造函数
	Accel_IMU_Raw(){
		this->frame_id = ACCEL_IMU_RAW_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_IMU_Raw(const _data_type& message_data){
		this->frame_id = ACCEL_IMU_RAW_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_IMU_Raw(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Accel_IMU_Raw(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AccelRawX : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelRawX(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -50;
			this->range_max_ = 50;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

	class AccelRawY : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelRawY(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -50;
			this->range_max_ = 50;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

	class AccelRawZ : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AccelRawZ(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.0001;
			this->offset_ = 0;
			this->range_min_ = -50;
			this->range_max_ = 50;
			this->unit_ = "g";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AccelRawX
		{
			_data_bin_type accelrawx_data_bin(this->accelrawx.data);
			for(uint8_t i = 0 ; i < this->accelrawx.signal_length() ; i++){
				this->data_bin[i + this->accelrawx.start_bit()] = accelrawx_data_bin[i];
			}
		}
		// 2. AccelRawY
		{
			_data_bin_type accelrawy_data_bin(this->accelrawy.data);
			for(uint8_t i = 0 ; i < this->accelrawy.signal_length() ; i++){
				this->data_bin[i + this->accelrawy.start_bit()] = accelrawy_data_bin[i];
			}
		}
		// 3. AccelRawZ
		{
			_data_bin_type accelrawz_data_bin(this->accelrawz.data);
			for(uint8_t i = 0 ; i < this->accelrawz.signal_length() ; i++){
				this->data_bin[i + this->accelrawz.start_bit()] = accelrawz_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AccelRawX
		{
			_data_bin_type accelrawx_data_bin;
			for(uint8_t i = 0; i < this->accelrawx.signal_length(); i++)
				accelrawx_data_bin[i] = this->data_bin[this->accelrawx.start_bit() + i];
			this->accelrawx.data = utils::bin2int(accelrawx_data_bin, this->accelrawx.signal_length());
		}
		// 2. AccelRawY
		{
			_data_bin_type accelrawy_data_bin;
			for(uint8_t i = 0; i < this->accelrawy.signal_length(); i++)
				accelrawy_data_bin[i] = this->data_bin[this->accelrawy.start_bit() + i];
			this->accelrawy.data = utils::bin2int(accelrawy_data_bin, this->accelrawy.signal_length());
		}
		// 3. AccelRawZ
		{
			_data_bin_type accelrawz_data_bin;
			for(uint8_t i = 0; i < this->accelrawz.signal_length(); i++)
				accelrawz_data_bin[i] = this->data_bin[this->accelrawz.start_bit() + i];
			this->accelrawz.data = utils::bin2int(accelrawz_data_bin, this->accelrawz.signal_length());
		}
	}

public: 
	AccelRawX accelrawx;
	AccelRawY accelrawy;
	AccelRawZ accelrawz;
};


class Ang_Rate_Raw_IMU : public CanMessage{
public: 
	// 构造函数
	Ang_Rate_Raw_IMU(){
		this->frame_id = ANG_RATE_RAW_IMU_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Ang_Rate_Raw_IMU(const _data_type& message_data){
		this->frame_id = ANG_RATE_RAW_IMU_FRAME_ID;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Ang_Rate_Raw_IMU(const _frame_id_type& id){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
	// 构造函数
	explicit Ang_Rate_Raw_IMU(const _frame_id_type& id , const _data_type& message_data){
		this->frame_id = id;
		this->message_length = 64;
		this->node_name = {"IMU"};
		this->data = message_data;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
	}
/************************ Signals Defination *****************************/
	class AngRateRawX : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateRawX(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 0;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

	class AngRateRawY : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateRawY(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 20;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

	class AngRateRawZ : public Signal<int32_t , double>{
	public: 
		// 构造函数
		AngRateRawZ(){
			this->data = 0;
			this->is_unsigned_ = false;
			this->start_bit_ = 40;
			this->signal_length_ = 20;
			this->is_intel_ = true;
			this->scale_ = 0.01;
			this->offset_ = 0;
			this->range_min_ = -500;
			this->range_max_ = 500;
			this->unit_ = "deg/s";
			this->receiver_ = {};
		}
	};

/*************************** END Defination ******************************/

public: 
	virtual void encode() override{
		// signals encode
		// 1. AngRateRawX
		{
			_data_bin_type angraterawx_data_bin(this->angraterawx.data);
			for(uint8_t i = 0 ; i < this->angraterawx.signal_length() ; i++){
				this->data_bin[i + this->angraterawx.start_bit()] = angraterawx_data_bin[i];
			}
		}
		// 2. AngRateRawY
		{
			_data_bin_type angraterawy_data_bin(this->angraterawy.data);
			for(uint8_t i = 0 ; i < this->angraterawy.signal_length() ; i++){
				this->data_bin[i + this->angraterawy.start_bit()] = angraterawy_data_bin[i];
			}
		}
		// 3. AngRateRawZ
		{
			_data_bin_type angraterawz_data_bin(this->angraterawz.data);
			for(uint8_t i = 0 ; i < this->angraterawz.signal_length() ; i++){
				this->data_bin[i + this->angraterawz.start_bit()] = angraterawz_data_bin[i];
			}
		}
		this->data = this->data_bin.to_ulong();
	}

private: 
	virtual void decode() override{
		// signals decode
		// 1. AngRateRawX
		{
			_data_bin_type angraterawx_data_bin;
			for(uint8_t i = 0; i < this->angraterawx.signal_length(); i++)
				angraterawx_data_bin[i] = this->data_bin[this->angraterawx.start_bit() + i];
			this->angraterawx.data = utils::bin2int(angraterawx_data_bin, this->angraterawx.signal_length());
		}
		// 2. AngRateRawY
		{
			_data_bin_type angraterawy_data_bin;
			for(uint8_t i = 0; i < this->angraterawy.signal_length(); i++)
				angraterawy_data_bin[i] = this->data_bin[this->angraterawy.start_bit() + i];
			this->angraterawy.data = utils::bin2int(angraterawy_data_bin, this->angraterawy.signal_length());
		}
		// 3. AngRateRawZ
		{
			_data_bin_type angraterawz_data_bin;
			for(uint8_t i = 0; i < this->angraterawz.signal_length(); i++)
				angraterawz_data_bin[i] = this->data_bin[this->angraterawz.start_bit() + i];
			this->angraterawz.data = utils::bin2int(angraterawz_data_bin, this->angraterawz.signal_length());
		}
	}

public: 
	AngRateRawX angraterawx;
	AngRateRawY angraterawy;
	AngRateRawZ angraterawz;
};


} // namespace gnss_ins

#endif
