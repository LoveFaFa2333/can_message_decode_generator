/*
 * @Descripttion: 
 * @Author: Gang Wang
 * @Date: 2023-01-30 09:30:45
 */
#include "can_message/gnss_ins_can_msg_decode_motorola.hpp"



int main(int argc , char* argv[]){
    gnss_ins::AngRateVehicle angRateVehicle;
    angRateVehicle.set_data("0000111111111111111111111111111111111111111111111111111111111111");
    angRateVehicle.angratex.set_data(0);
    angRateVehicle.angratey.set_data(0.01);
    angRateVehicle.angratez.set_data(-0.01);
    angRateVehicle.encode();

    std::cout << angRateVehicle.get_data_bin()<< std::endl;
    std::cout << angRateVehicle.angratex.raw_data() << angRateVehicle.angratex.unit()<< std::endl;
    std::cout << angRateVehicle.angratey.raw_data() << angRateVehicle.angratey.unit()<< std::endl;
    std::cout << angRateVehicle.angratez.raw_data() << angRateVehicle.angratez.unit()<< std::endl;

    gnss_ins::Longitude longitude;
    std::array<uint8_t , 8> array_data = {0,0,0,0,0,0,0,3};
    longitude.set_data(array_data);
    std::cout << longitude.get_data_bin()<< std::endl;
    std::cout << longitude.poslon2.raw_data() << longitude.poslon2.unit()<< std::endl;
    for(int i = 0 ; i < 8 ; i++)
        std::cout << (int)longitude.get_data_array().at(i) << " ";
    std::cout << std::endl;


    gnss_ins::HeadingPitchRoll headingPitchRoll;
    headingPitchRoll.angleroll.set_data(0.02);
    headingPitchRoll.anglepitch.set_data(-0.01);
    headingPitchRoll.angleheading.set_data(0.04);
    headingPitchRoll.encode();

    headingPitchRoll.set_data("000100000000101011111111111111111000000000000100");

    std::cout << headingPitchRoll.get_data_bin()<< std::endl;
    std::cout << headingPitchRoll.angleroll.raw_data() << headingPitchRoll.angleroll.unit()<< std::endl;
    std::cout << headingPitchRoll.anglepitch.raw_data() << headingPitchRoll.anglepitch.unit()<< std::endl;
    std::cout << headingPitchRoll.angleheading.raw_data() << headingPitchRoll.angleheading.unit()<< std::endl;


    gnss_ins::Accel_IMU_Raw accelIMURaw;
    // std::array<uint8_t , 8> array_data = {255,255,16,0,1,0,0,48};
    accelIMURaw.accelrawx.set_data(-0.0015);
    accelIMURaw.accelrawy.set_data(0.0001);
    accelIMURaw.accelrawz.set_data(0.0003);
    accelIMURaw.encode();

    std::cout << accelIMURaw.get_data_bin()<< std::endl;
    std::cout << accelIMURaw.accelrawx.raw_data() << accelIMURaw.accelrawx.unit()<< std::endl;
    std::cout << accelIMURaw.accelrawy.raw_data() << accelIMURaw.accelrawy.unit()<< std::endl;
    std::cout << accelIMURaw.accelrawz.raw_data() << accelIMURaw.accelrawz.unit()<< std::endl;

    return 0;
}