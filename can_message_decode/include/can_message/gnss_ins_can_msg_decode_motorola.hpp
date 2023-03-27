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
			this->start_bit_ = 7;
			this->signal_length_ = 64;
			this->is_intel_ = false;
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
			uint8_t index = this->poslat2.start_bit() , length = this->poslat2.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = poslat2_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = poslat2.signal_length()-1;
			uint8_t j = this->poslat2.start_bit();
			while(index <= length){
				poslat2_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->poslat2.data = utils::bin2int(poslat2_data_bin, this->poslat2.signal_length(), this->poslat2.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 64;
			this->is_intel_ = false;
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
			uint8_t index = this->poslon2.start_bit() , length = this->poslon2.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = poslon2_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = poslon2.signal_length()-1;
			uint8_t j = this->poslon2.start_bit();
			while(index <= length){
				poslon2_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->poslon2.data = utils::bin2int(poslon2_data_bin, this->poslon2.signal_length(), this->poslon2.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->angratex.start_bit() , length = this->angratex.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angratex_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AngRateY
		{
			_data_bin_type angratey_data_bin(this->angratey.data);
			uint8_t index = this->angratey.start_bit() , length = this->angratey.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angratey_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AngRateZ
		{
			_data_bin_type angratez_data_bin(this->angratez.data);
			uint8_t index = this->angratez.start_bit() , length = this->angratez.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angratez_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = angratex.signal_length()-1;
			uint8_t j = this->angratex.start_bit();
			while(index <= length){
				angratex_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angratex.data = utils::bin2int(angratex_data_bin, this->angratex.signal_length(), this->angratex.is_unsigned());
		}
		// 2. AngRateY
		{
			_data_bin_type angratey_data_bin;
			uint8_t index = 0 ,  length = angratey.signal_length()-1;
			uint8_t j = this->angratey.start_bit();
			while(index <= length){
				angratey_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angratey.data = utils::bin2int(angratey_data_bin, this->angratey.signal_length(), this->angratey.is_unsigned());
		}
		// 3. AngRateZ
		{
			_data_bin_type angratez_data_bin;
			uint8_t index = 0 ,  length = angratez.signal_length()-1;
			uint8_t j = this->angratez.start_bit();
			while(index <= length){
				angratez_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angratez.data = utils::bin2int(angratez_data_bin, this->angratez.signal_length(), this->angratez.is_unsigned());
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
		AngleHeadingSigma(){
			this->data = 0;
			this->is_unsigned_ = true;
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->angleheadingsigma.start_bit() , length = this->angleheadingsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angleheadingsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AnglePitchSigma
		{
			_data_bin_type anglepitchsigma_data_bin(this->anglepitchsigma.data);
			uint8_t index = this->anglepitchsigma.start_bit() , length = this->anglepitchsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = anglepitchsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AngleRollSigma
		{
			_data_bin_type anglerollsigma_data_bin(this->anglerollsigma.data);
			uint8_t index = this->anglerollsigma.start_bit() , length = this->anglerollsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = anglerollsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = angleheadingsigma.signal_length()-1;
			uint8_t j = this->angleheadingsigma.start_bit();
			while(index <= length){
				angleheadingsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angleheadingsigma.data = utils::bin2int(angleheadingsigma_data_bin, this->angleheadingsigma.signal_length(), this->angleheadingsigma.is_unsigned());
		}
		// 2. AnglePitchSigma
		{
			_data_bin_type anglepitchsigma_data_bin;
			uint8_t index = 0 ,  length = anglepitchsigma.signal_length()-1;
			uint8_t j = this->anglepitchsigma.start_bit();
			while(index <= length){
				anglepitchsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->anglepitchsigma.data = utils::bin2int(anglepitchsigma_data_bin, this->anglepitchsigma.signal_length(), this->anglepitchsigma.is_unsigned());
		}
		// 3. AngleRollSigma
		{
			_data_bin_type anglerollsigma_data_bin;
			uint8_t index = 0 ,  length = anglerollsigma.signal_length()-1;
			uint8_t j = this->anglerollsigma.start_bit();
			while(index <= length){
				anglerollsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->anglerollsigma.data = utils::bin2int(anglerollsigma_data_bin, this->anglerollsigma.signal_length(), this->anglerollsigma.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 23;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 39;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			uint8_t index = this->angleheading.start_bit() , length = this->angleheading.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angleheading_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AnglePitch
		{
			_data_bin_type anglepitch_data_bin(this->anglepitch.data);
			uint8_t index = this->anglepitch.start_bit() , length = this->anglepitch.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = anglepitch_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AngleRoll
		{
			_data_bin_type angleroll_data_bin(this->angleroll.data);
			uint8_t index = this->angleroll.start_bit() , length = this->angleroll.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angleroll_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = angleheading.signal_length()-1;
			uint8_t j = this->angleheading.start_bit();
			while(index <= length){
				angleheading_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angleheading.data = utils::bin2int(angleheading_data_bin, this->angleheading.signal_length(), this->angleheading.is_unsigned());
		}
		// 2. AnglePitch
		{
			_data_bin_type anglepitch_data_bin;
			uint8_t index = 0 ,  length = anglepitch.signal_length()-1;
			uint8_t j = this->anglepitch.start_bit();
			while(index <= length){
				anglepitch_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->anglepitch.data = utils::bin2int(anglepitch_data_bin, this->anglepitch.signal_length(), this->anglepitch.is_unsigned());
		}
		// 3. AngleRoll
		{
			_data_bin_type angleroll_data_bin;
			uint8_t index = 0 ,  length = angleroll.signal_length()-1;
			uint8_t j = this->angleroll.start_bit();
			while(index <= length){
				angleroll_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angleroll.data = utils::bin2int(angleroll_data_bin, this->angleroll.signal_length(), this->angleroll.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->accelx.start_bit() , length = this->accelx.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accelx_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AccelY
		{
			_data_bin_type accely_data_bin(this->accely.data);
			uint8_t index = this->accely.start_bit() , length = this->accely.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accely_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AccelZ
		{
			_data_bin_type accelz_data_bin(this->accelz.data);
			uint8_t index = this->accelz.start_bit() , length = this->accelz.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accelz_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = accelx.signal_length()-1;
			uint8_t j = this->accelx.start_bit();
			while(index <= length){
				accelx_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accelx.data = utils::bin2int(accelx_data_bin, this->accelx.signal_length(), this->accelx.is_unsigned());
		}
		// 2. AccelY
		{
			_data_bin_type accely_data_bin;
			uint8_t index = 0 ,  length = accely.signal_length()-1;
			uint8_t j = this->accely.start_bit();
			while(index <= length){
				accely_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accely.data = utils::bin2int(accely_data_bin, this->accely.signal_length(), this->accely.is_unsigned());
		}
		// 3. AccelZ
		{
			_data_bin_type accelz_data_bin;
			uint8_t index = 0 ,  length = accelz.signal_length()-1;
			uint8_t j = this->accelz.start_bit();
			while(index <= length){
				accelz_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accelz.data = utils::bin2int(accelz_data_bin, this->accelz.signal_length(), this->accelz.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 23;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 39;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 55;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			uint8_t index = this->velesigma.start_bit() , length = this->velesigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = velesigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. VelNSigma
		{
			_data_bin_type velnsigma_data_bin(this->velnsigma.data);
			uint8_t index = this->velnsigma.start_bit() , length = this->velnsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = velnsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. VelUSigma
		{
			_data_bin_type velusigma_data_bin(this->velusigma.data);
			uint8_t index = this->velusigma.start_bit() , length = this->velusigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = velusigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 4. VelSigma
		{
			_data_bin_type velsigma_data_bin(this->velsigma.data);
			uint8_t index = this->velsigma.start_bit() , length = this->velsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = velsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = velesigma.signal_length()-1;
			uint8_t j = this->velesigma.start_bit();
			while(index <= length){
				velesigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->velesigma.data = utils::bin2int(velesigma_data_bin, this->velesigma.signal_length(), this->velesigma.is_unsigned());
		}
		// 2. VelNSigma
		{
			_data_bin_type velnsigma_data_bin;
			uint8_t index = 0 ,  length = velnsigma.signal_length()-1;
			uint8_t j = this->velnsigma.start_bit();
			while(index <= length){
				velnsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->velnsigma.data = utils::bin2int(velnsigma_data_bin, this->velnsigma.signal_length(), this->velnsigma.is_unsigned());
		}
		// 3. VelUSigma
		{
			_data_bin_type velusigma_data_bin;
			uint8_t index = 0 ,  length = velusigma.signal_length()-1;
			uint8_t j = this->velusigma.start_bit();
			while(index <= length){
				velusigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->velusigma.data = utils::bin2int(velusigma_data_bin, this->velusigma.signal_length(), this->velusigma.is_unsigned());
		}
		// 4. VelSigma
		{
			_data_bin_type velsigma_data_bin;
			uint8_t index = 0 ,  length = velsigma.signal_length()-1;
			uint8_t j = this->velsigma.start_bit();
			while(index <= length){
				velsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->velsigma.data = utils::bin2int(velsigma_data_bin, this->velsigma.signal_length(), this->velsigma.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 23;
			this->signal_length_ = 32;
			this->is_intel_ = false;
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
			uint8_t index = this->gpsweek.start_bit() , length = this->gpsweek.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsweek_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. GpsTime
		{
			_data_bin_type gpstime_data_bin(this->gpstime.data);
			uint8_t index = this->gpstime.start_bit() , length = this->gpstime.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpstime_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = gpsweek.signal_length()-1;
			uint8_t j = this->gpsweek.start_bit();
			while(index <= length){
				gpsweek_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsweek.data = utils::bin2int(gpsweek_data_bin, this->gpsweek.signal_length(), this->gpsweek.is_unsigned());
		}
		// 2. GpsTime
		{
			_data_bin_type gpstime_data_bin;
			uint8_t index = 0 ,  length = gpstime.signal_length()-1;
			uint8_t j = this->gpstime.start_bit();
			while(index <= length){
				gpstime_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpstime.data = utils::bin2int(gpstime_data_bin, this->gpstime.signal_length(), this->gpstime.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 23;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 39;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 55;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			uint8_t index = this->vele.start_bit() , length = this->vele.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = vele_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. VelN
		{
			_data_bin_type veln_data_bin(this->veln.data);
			uint8_t index = this->veln.start_bit() , length = this->veln.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = veln_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. VelU
		{
			_data_bin_type velu_data_bin(this->velu.data);
			uint8_t index = this->velu.start_bit() , length = this->velu.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = velu_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 4. Vel
		{
			_data_bin_type vel_data_bin(this->vel.data);
			uint8_t index = this->vel.start_bit() , length = this->vel.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = vel_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = vele.signal_length()-1;
			uint8_t j = this->vele.start_bit();
			while(index <= length){
				vele_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->vele.data = utils::bin2int(vele_data_bin, this->vele.signal_length(), this->vele.is_unsigned());
		}
		// 2. VelN
		{
			_data_bin_type veln_data_bin;
			uint8_t index = 0 ,  length = veln.signal_length()-1;
			uint8_t j = this->veln.start_bit();
			while(index <= length){
				veln_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->veln.data = utils::bin2int(veln_data_bin, this->veln.signal_length(), this->veln.is_unsigned());
		}
		// 3. VelU
		{
			_data_bin_type velu_data_bin;
			uint8_t index = 0 ,  length = velu.signal_length()-1;
			uint8_t j = this->velu.start_bit();
			while(index <= length){
				velu_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->velu.data = utils::bin2int(velu_data_bin, this->velu.signal_length(), this->velu.is_unsigned());
		}
		// 4. Vel
		{
			_data_bin_type vel_data_bin;
			uint8_t index = 0 ,  length = vel.signal_length()-1;
			uint8_t j = this->vel.start_bit();
			while(index <= length){
				vel_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->vel.data = utils::bin2int(vel_data_bin, this->vel.signal_length(), this->vel.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->posesigma.start_bit() , length = this->posesigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = posesigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. PosNsigma
		{
			_data_bin_type posnsigma_data_bin(this->posnsigma.data);
			uint8_t index = this->posnsigma.start_bit() , length = this->posnsigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = posnsigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. PosUsigma
		{
			_data_bin_type posusigma_data_bin(this->posusigma.data);
			uint8_t index = this->posusigma.start_bit() , length = this->posusigma.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = posusigma_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = posesigma.signal_length()-1;
			uint8_t j = this->posesigma.start_bit();
			while(index <= length){
				posesigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->posesigma.data = utils::bin2int(posesigma_data_bin, this->posesigma.signal_length(), this->posesigma.is_unsigned());
		}
		// 2. PosNsigma
		{
			_data_bin_type posnsigma_data_bin;
			uint8_t index = 0 ,  length = posnsigma.signal_length()-1;
			uint8_t j = this->posnsigma.start_bit();
			while(index <= length){
				posnsigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->posnsigma.data = utils::bin2int(posnsigma_data_bin, this->posnsigma.signal_length(), this->posnsigma.is_unsigned());
		}
		// 3. PosUsigma
		{
			_data_bin_type posusigma_data_bin;
			uint8_t index = 0 ,  length = posusigma.signal_length()-1;
			uint8_t j = this->posusigma.start_bit();
			while(index <= length){
				posusigma_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->posusigma.data = utils::bin2int(posusigma_data_bin, this->posusigma.signal_length(), this->posusigma.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 32;
			this->is_intel_ = false;
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
			uint8_t index = this->posalt.start_bit() , length = this->posalt.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = posalt_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = posalt.signal_length()-1;
			uint8_t j = this->posalt.start_bit();
			while(index <= length){
				posalt_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->posalt.data = utils::bin2int(posalt_data_bin, this->posalt.signal_length(), this->posalt.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 32;
			this->is_intel_ = false;
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
			this->start_bit_ = 39;
			this->signal_length_ = 32;
			this->is_intel_ = false;
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
			uint8_t index = this->poslat.start_bit() , length = this->poslat.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = poslat_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. PosLon
		{
			_data_bin_type poslon_data_bin(this->poslon.data);
			uint8_t index = this->poslon.start_bit() , length = this->poslon.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = poslon_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = poslat.signal_length()-1;
			uint8_t j = this->poslat.start_bit();
			while(index <= length){
				poslat_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->poslat.data = utils::bin2int(poslat_data_bin, this->poslat.signal_length(), this->poslat.is_unsigned());
		}
		// 2. PosLon
		{
			_data_bin_type poslon_data_bin;
			uint8_t index = 0 ,  length = poslon.signal_length()-1;
			uint8_t j = this->poslon.start_bit();
			while(index <= length){
				poslon_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->poslon.data = utils::bin2int(poslon_data_bin, this->poslon.signal_length(), this->poslon.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			this->start_bit_ = 15;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			this->start_bit_ = 23;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			this->start_bit_ = 31;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			this->start_bit_ = 39;
			this->signal_length_ = 16;
			this->is_intel_ = false;
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
			this->start_bit_ = 55;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			this->start_bit_ = 63;
			this->signal_length_ = 8;
			this->is_intel_ = false;
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
			uint8_t index = this->system_state.start_bit() , length = this->system_state.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = system_state_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. GpsNumSatsUsed
		{
			_data_bin_type gpsnumsatsused_data_bin(this->gpsnumsatsused.data);
			uint8_t index = this->gpsnumsatsused.start_bit() , length = this->gpsnumsatsused.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsnumsatsused_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. satellite_status
		{
			_data_bin_type satellite_status_data_bin(this->satellite_status.data);
			uint8_t index = this->satellite_status.start_bit() , length = this->satellite_status.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = satellite_status_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 4. GpsNumSats2Used
		{
			_data_bin_type gpsnumsats2used_data_bin(this->gpsnumsats2used.data);
			uint8_t index = this->gpsnumsats2used.start_bit() , length = this->gpsnumsats2used.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsnumsats2used_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 5. GpsAge
		{
			_data_bin_type gpsage_data_bin(this->gpsage.data);
			uint8_t index = this->gpsage.start_bit() , length = this->gpsage.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsage_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 6. GpsNumSats
		{
			_data_bin_type gpsnumsats_data_bin(this->gpsnumsats.data);
			uint8_t index = this->gpsnumsats.start_bit() , length = this->gpsnumsats.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsnumsats_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 7. GpsNumSats2
		{
			_data_bin_type gpsnumsats2_data_bin(this->gpsnumsats2.data);
			uint8_t index = this->gpsnumsats2.start_bit() , length = this->gpsnumsats2.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = gpsnumsats2_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = system_state.signal_length()-1;
			uint8_t j = this->system_state.start_bit();
			while(index <= length){
				system_state_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->system_state.data = utils::bin2int(system_state_data_bin, this->system_state.signal_length(), this->system_state.is_unsigned());
		}
		// 2. GpsNumSatsUsed
		{
			_data_bin_type gpsnumsatsused_data_bin;
			uint8_t index = 0 ,  length = gpsnumsatsused.signal_length()-1;
			uint8_t j = this->gpsnumsatsused.start_bit();
			while(index <= length){
				gpsnumsatsused_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsnumsatsused.data = utils::bin2int(gpsnumsatsused_data_bin, this->gpsnumsatsused.signal_length(), this->gpsnumsatsused.is_unsigned());
		}
		// 3. satellite_status
		{
			_data_bin_type satellite_status_data_bin;
			uint8_t index = 0 ,  length = satellite_status.signal_length()-1;
			uint8_t j = this->satellite_status.start_bit();
			while(index <= length){
				satellite_status_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->satellite_status.data = utils::bin2int(satellite_status_data_bin, this->satellite_status.signal_length(), this->satellite_status.is_unsigned());
		}
		// 4. GpsNumSats2Used
		{
			_data_bin_type gpsnumsats2used_data_bin;
			uint8_t index = 0 ,  length = gpsnumsats2used.signal_length()-1;
			uint8_t j = this->gpsnumsats2used.start_bit();
			while(index <= length){
				gpsnumsats2used_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsnumsats2used.data = utils::bin2int(gpsnumsats2used_data_bin, this->gpsnumsats2used.signal_length(), this->gpsnumsats2used.is_unsigned());
		}
		// 5. GpsAge
		{
			_data_bin_type gpsage_data_bin;
			uint8_t index = 0 ,  length = gpsage.signal_length()-1;
			uint8_t j = this->gpsage.start_bit();
			while(index <= length){
				gpsage_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsage.data = utils::bin2int(gpsage_data_bin, this->gpsage.signal_length(), this->gpsage.is_unsigned());
		}
		// 6. GpsNumSats
		{
			_data_bin_type gpsnumsats_data_bin;
			uint8_t index = 0 ,  length = gpsnumsats.signal_length()-1;
			uint8_t j = this->gpsnumsats.start_bit();
			while(index <= length){
				gpsnumsats_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsnumsats.data = utils::bin2int(gpsnumsats_data_bin, this->gpsnumsats.signal_length(), this->gpsnumsats.is_unsigned());
		}
		// 7. GpsNumSats2
		{
			_data_bin_type gpsnumsats2_data_bin;
			uint8_t index = 0 ,  length = gpsnumsats2.signal_length()-1;
			uint8_t j = this->gpsnumsats2.start_bit();
			while(index <= length){
				gpsnumsats2_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->gpsnumsats2.data = utils::bin2int(gpsnumsats2_data_bin, this->gpsnumsats2.signal_length(), this->gpsnumsats2.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->accelrawx.start_bit() , length = this->accelrawx.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accelrawx_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AccelRawY
		{
			_data_bin_type accelrawy_data_bin(this->accelrawy.data);
			uint8_t index = this->accelrawy.start_bit() , length = this->accelrawy.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accelrawy_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AccelRawZ
		{
			_data_bin_type accelrawz_data_bin(this->accelrawz.data);
			uint8_t index = this->accelrawz.start_bit() , length = this->accelrawz.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = accelrawz_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = accelrawx.signal_length()-1;
			uint8_t j = this->accelrawx.start_bit();
			while(index <= length){
				accelrawx_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accelrawx.data = utils::bin2int(accelrawx_data_bin, this->accelrawx.signal_length(), this->accelrawx.is_unsigned());
		}
		// 2. AccelRawY
		{
			_data_bin_type accelrawy_data_bin;
			uint8_t index = 0 ,  length = accelrawy.signal_length()-1;
			uint8_t j = this->accelrawy.start_bit();
			while(index <= length){
				accelrawy_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accelrawy.data = utils::bin2int(accelrawy_data_bin, this->accelrawy.signal_length(), this->accelrawy.is_unsigned());
		}
		// 3. AccelRawZ
		{
			_data_bin_type accelrawz_data_bin;
			uint8_t index = 0 ,  length = accelrawz.signal_length()-1;
			uint8_t j = this->accelrawz.start_bit();
			while(index <= length){
				accelrawz_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->accelrawz.data = utils::bin2int(accelrawz_data_bin, this->accelrawz.signal_length(), this->accelrawz.is_unsigned());
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
			this->start_bit_ = 7;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 19;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			this->start_bit_ = 47;
			this->signal_length_ = 20;
			this->is_intel_ = false;
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
			uint8_t index = this->angraterawx.start_bit() , length = this->angraterawx.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angraterawx_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 2. AngRateRawY
		{
			_data_bin_type angraterawy_data_bin(this->angraterawy.data);
			uint8_t index = this->angraterawy.start_bit() , length = this->angraterawy.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angraterawy_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
			}
		}
		// 3. AngRateRawZ
		{
			_data_bin_type angraterawz_data_bin(this->angraterawz.data);
			uint8_t index = this->angraterawz.start_bit() , length = this->angraterawz.signal_length() - 1;
			for(uint8_t i = 0 ; i <= length ; i++){
				this->data_bin[index] = angraterawz_data_bin[length - i];
				index += (index%8 == 0)? 15 : -1;
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
			uint8_t index = 0 ,  length = angraterawx.signal_length()-1;
			uint8_t j = this->angraterawx.start_bit();
			while(index <= length){
				angraterawx_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angraterawx.data = utils::bin2int(angraterawx_data_bin, this->angraterawx.signal_length(), this->angraterawx.is_unsigned());
		}
		// 2. AngRateRawY
		{
			_data_bin_type angraterawy_data_bin;
			uint8_t index = 0 ,  length = angraterawy.signal_length()-1;
			uint8_t j = this->angraterawy.start_bit();
			while(index <= length){
				angraterawy_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angraterawy.data = utils::bin2int(angraterawy_data_bin, this->angraterawy.signal_length(), this->angraterawy.is_unsigned());
		}
		// 3. AngRateRawZ
		{
			_data_bin_type angraterawz_data_bin;
			uint8_t index = 0 ,  length = angraterawz.signal_length()-1;
			uint8_t j = this->angraterawz.start_bit();
			while(index <= length){
				angraterawz_data_bin[length - index] = this->data_bin[j];
				j += (j%8 == 0)? 15 : -1;
				index++;
			}
			this->angraterawz.data = utils::bin2int(angraterawz_data_bin, this->angraterawz.signal_length(), this->angraterawz.is_unsigned());
		}
	}

public: 
	AngRateRawX angraterawx;
	AngRateRawY angraterawy;
	AngRateRawZ angraterawz;
};


} // namespace gnss_ins

#endif