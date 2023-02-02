#ifndef GNSS_INS_HPP_
#define GNSS_INS_HPP_

#include <algorithm>

#include "can_message/can_message.hpp"
#include "can_message/signal.hpp"
#include "utils/utils.hpp"

/* Frame ids. */
#define LATITUDE_FRAME_ID (0x32eu)
#define LONGITUDE_FRAME_ID (0x32du)
#define ANG_RATE_VEHICLE_FRAME_ID (0x32cu)
#define HEADING_PITCH_ROLL_SIGMA_FRAME_ID (0x32bu)
#define HEADING_PITCH_ROLL_FRAME_ID (0x32au)
#define ACCEL_VEHICLE_FRAME_ID (0x329u)
#define VELOCITY_LEVEL_SIGMA_FRAME_ID (0x328u)
#define TIME_FRAME_ID (0x320u)
#define VELOCITY_LEVEL_FRAME_ID (0x327u)
#define POS_SIGMA_FRAME_ID (0x326u)
#define ALTITUDE_FRAME_ID (0x325u)
#define LATITUDE_LONGITUDE_FRAME_ID (0x324u)
#define INS_STATUS_FRAME_ID (0x323u)
#define ACCEL_IMU_RAW_FRAME_ID (0x322u)
#define ANG_RATE_RAW_IMU_FRAME_ID (0x321u)

namespace gnss_ins{

/*************************** Message Defination *******************************/

class Longitude : public CanMessage{
public:
    Longitude(){
        this->frame_id = LONGITUDE_FRAME_ID;
        this->message_length = 64;
        this->node_name = {""};
        this->data = 0;
        this->data_bin = _data_bin_type(this->data);
        this->decode();
    }


    explicit Longitude(const _data_type& message_data){
        this->frame_id = LONGITUDE_FRAME_ID;
        this->message_length = 64;
        this->node_name = {""};
        this->data = message_data;
        this->data_bin = _data_bin_type(this->data);
        this->decode();
    };

    explicit Longitude(const _frame_id_type& id): frame_id(id){
		this->message_length = 64;
		this->node_name = {""};
        this->data = 0;
		this->data_bin = _data_bin_type(this->data);
		this->decode();
    }

    explicit Longitude(const _frame_id_type& id , const _data_type& message_data): frame_id(id) , data(message_data){
		this->message_length = 64;
		this->node_name = {""};
		this->data_bin = _data_bin_type(this->data);
		this->decode();
    }

/************************ Signal Defination START *****************************/
    class PosLon2 : public Signal<int64_t , double>{
    public:
        PosLon2(){
            this->data = 0;
            this->is_unsigned_ = false;
            this->start_bit_ = 0;
            this->signal_length_ = 64;
            this->is_intel_ = true;
            this->end_bit_ = this->start_bit_ + this->signal_length_ -1;
            this->scale_ = 1.0e-8;
            this->offset_ = 0.0;
            this->range_min_ = -180;
            this->range_max_ = 180;
            this->unit_ = "deg";
            this->receiver_ = {""};
        }

    };
    /************************ Signals Defination END ******************************/
public:
    virtual void encode() override{
        if(pos_lon2.byte_oder()){ //intel
            // signals encode
            // 1. pos_lon2
            _data_bin_type pos_lon2_data_bin(this->pos_lon2.data);
            for(uint8_t i = 0 ; i < this->pos_lon2.signal_length() ; i++){
                this->data_bin[this->pos_lon2.start_bit() + i] = pos_lon2_data_bin[i];
            }

            // ...
        }else { // motorola MSB
            _data_bin_type pos_lon2_data_bin(this->pos_lon2.data);
            uint8_t index = this->pos_lon2.start_bit() , length = this->pos_lon2.signal_length() - 1;
            for(uint8_t i = 0 ; i <= length ; i++){
                this->data_bin[index] = pos_lon2_data_bin[length - i];
                index += (index%8 == 0)? 15 : -1;
            }

            // ...
        }

        this->data = this->data_bin.to_ulong();

    }

private:
    virtual void decode() override{
        // 判断编码方式
        if(pos_lon2.byte_oder()){ //intel

        // signals decode
        // 1. pos_lon2
            _data_bin_type pos_lon2_data_bin;
            for(int i = 0; i < this->pos_lon2.signal_length(); i++)
                pos_lon2_data_bin[i] = this->data_bin[this->pos_lon2.start_bit() + i];
            this->pos_lon2.data = utils::bin2int(pos_lon2_data_bin , this->pos_lon2.signal_length());

        //...
        
        }esle { // motorola MSB
            _data_bin_type pos_lon2_data_bin;
            int index = 0 ,  length = pos_lon2.signal_length()-1;
            int j = this->pos_lon2.start_bit();
            while(index <= length){
                pos_lon2_data_bin[length - index] = this->data_bin[j];
                j += (j%8 == 0)? 15 : -1;
                index++;
            }
            this->pos_lon2.data = utils::bin2int(pos_lon2_data_bin , this->pos_lon2.signal_length());

        //...    
        }
    }

public:
    PosLon2 pos_lon2;
};
/************************ Message Defination END ****************************/

}       // namespace gnss_ins
#endif