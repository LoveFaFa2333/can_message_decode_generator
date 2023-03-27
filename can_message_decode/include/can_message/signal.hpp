/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-01-17 17:38:10
 */

#ifndef SIGNAL_HPP_
#define SIGNAL_HPP_

#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>
#include <bitset>

template <typename _data_type , typename _scale_type>
class Signal{
public:
    Signal(){};

    explicit Signal(const _data_type &data_set): data(data_set){};


    bool is_out_of_range(const _scale_type& data){
        if(this->signal_length_ == 1){
            return !(data == (_scale_type)1 || data == (_scale_type)0);
        }
        
        return data > this->range_max_ || data < this->range_min_;
    }

    _scale_type raw_data(){
        _scale_type raw_data = this->data * this->scale_ + this->offset_;

        if(is_out_of_range(raw_data)){
            std::cout << "Error : signal data is out of range , cat not return raw data , data reset to 0, exit ..." << std::endl;

            this->data = 0;
            // exit(0);
        }
        return raw_data;
    }

    // setter
    void set_data(const std::string& bin_string){
        std::bitset<sizeof(_data_type) * 8> data_bin(bin_string);
        this->data = data_bin.to_ulong();
    }

    void set_data(const _data_type& data){
        this->data = data;
    }

    void set_data(const _scale_type& data){
        if(is_out_of_range(data)){
            std::cout << "Error : signal data is out of range , exit ..." << std::endl;
            // exit(0);
        }

        std::string str = std::to_string((data - this->offset_) / this->scale_);
        this->data = atoi(str.c_str());
    }

    //getter
    bool is_unsigned(){
        return this->is_unsigned_;
    }

    uint8_t start_bit(){
        return this->start_bit_;
    }

    uint8_t signal_length(){
        return this->signal_length_;
    }

    _scale_type scale(){
        return this->scale_;
    }

    _scale_type offset(){
        return this->offset_;
    }

    _scale_type range_min(){
        return this->range_min_;
    }

    _scale_type range_max(){
        return this->range_max_;
    }

    bool byte_oder(){
        return this->byte_oder;
    }

    std::string unit(){
        return this->unit_;
    }

    std::vector<std::string> receiver(){
        return this->receiver_;
    }

public:
    _data_type data;

protected:
    bool is_unsigned_;
    uint8_t start_bit_;
    uint8_t signal_length_;
    bool is_intel_;
    _scale_type scale_;
    _scale_type offset_;
    _scale_type range_min_;
    _scale_type range_max_;
    std::string unit_;
    std::vector<std::string> receiver_;
};

#endif