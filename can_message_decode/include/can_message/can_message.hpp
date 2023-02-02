#ifndef CAN_MESSAGE_HPP_
#define CAN_MESSAGE_HPP_

#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>
#include <bitset>
#include <array>
#include <cmath>

/* can message defination */
class CanMessage{

public:
    using _frame_id_type = uint32_t;
    using _message_length_type = uint8_t;
    using _node_name_type = std::vector<std::string>;
    using _data_type = uint64_t;
    using _data_bin_type = std::bitset<sizeof(_data_type) * 8>;
    using _data_array_type = std::array<uint8_t , 8>;

    /* get parameters */
    _frame_id_type get_frame_id() const{
        return this->frame_id;
    }

    _message_length_type get_dlc() const{
        return this->message_length;
    }

    std::vector<std::string> get_node_name() const{
        return this->node_name;
    }

    _data_type get_data() const{
        return this->data;
    }

    _data_bin_type get_data_bin() const{
        return this->data_bin;
    }

    _data_array_type get_data_array(){
        _data_array_type data_array;
        for(int i = 0 ; i < 8 ; i++){
            std::bitset<8> byte_data;
            for(int j = 0 ; j < 8 ; j++)
                byte_data[j] = this->data_bin[i * 8 + j];

            data_array[i] = static_cast<uint8_t>(byte_data.to_ulong());
        }
        return data_array;
    }    


    /* set parameters */
    void set_frame_id(const _data_type& id){
        this->frame_id = id;
    }

    void set_dlc(const _message_length_type dlc){
        this->message_length = dlc;
    }

    void set_node_name(const _node_name_type& node){
        this->node_name = node;
    }

    // set use uint type
    void set_data(const _data_type& message_data){
        this->data = message_data;
        this->data_bin = _data_bin_type(this->data);
        decode();
    }

    // set use string type
    void set_data(const std::string& message_data_bin){
        this->data_bin = _data_bin_type(message_data_bin);
        this->data = this->data_bin.to_ulong();
        decode();
    }

    /*  set use std::array<uint8_t , 8> type */
    void set_data(const _data_array_type& message_data_array){
        for(uint8_t i = 0 ; i < message_data_array.size() ; i++){
            auto byte_data = message_data_array[i];
            std::bitset<8> byte_data_bin(byte_data);
            for(uint8_t j = 0 ;  j < 8 ; j++){
                int index = i * 8 + j;
                this->data_bin[index] = byte_data_bin[j];
            }
        }

        this->data = this->data_bin.to_ulong();
        decode();
    }


protected:
    virtual void decode() = 0;
    virtual void encode() = 0;

protected:
        /* frame_id */
        _frame_id_type frame_id;

        /* message length (bits) */
        _message_length_type message_length;

        /* sender node name */
        _node_name_type node_name;

        /* uint data  of frame */
        _data_type data;

        /* binary data  of frame */
        _data_bin_type data_bin;

        /* Signal1 defination */
        /* example :
            AngRateZ ang_rate_z;
            AngRateY ang_rate_y;
            AngRatex ang_rate_x;
        */

};

#endif