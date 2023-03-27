/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-01-30 09:30:45
 */
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <iostream>
#include <cmath>
#include <bitset>


namespace utils{


// /* 使用bitset将int 转为 二进制
//    可选为  */
template <typename _type>
const std::bitset<sizeof(_type) * 8> int2bin(_type data){
    constexpr const uint8_t bits = sizeof(_type) * 8;
    bool is_negative = data;
    std::bitset<bits> bin;

    if(is_negative){
        bin = std::bitset<bits>(-data);
        bin.flip(); //所有位取反
        uint8_t index = 0;
        bool if_carry = true;
        // 末尾+1 进行补码
        while(if_carry && index < bits){
            if_carry = bin[index];
            bin[index] = ~bin[index];
            index++;
        }
    }else{
        bin = std::bitset<bits>(data);
    }

    return bin;
}


/* 使用bitset将二进制 转为 int类型 */
/* length 为二进制数据长度 */
int64_t bin2int(std::bitset<sizeof(int64_t) * 8>  bin , const uint8_t& length , const bool &is_unsigned){
    if(length == 8 || length == 16 || length == 32 || length == 64)
        return bin.to_ulong();

    int64_t data = 0;
    if(is_unsigned || bin[length-1]){
        data = (int64_t)bin.to_ulong();
    }else{
        
        bool if_down = true;
        uint8_t index = 0;
        // 末尾-1
        while(if_down && index < length){
            if_down = !bin[index];
            bin[index] = ~bin[index];
            index++;
        }

        // 取反
        index = 0;
        while(index < length)
            bin.flip(index++);

        // 得到负数
		// std::cout << bin << std::endl;
        data = -(int64_t)bin.to_ulong();

    }
    
    return data;
}

} // namespace utils


#endif