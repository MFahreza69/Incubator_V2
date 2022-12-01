#include <Arduino.h>
#include <EEPROM.h>

#include "Xbaby.h"

Xbaby::Xbaby(){
    pos = 0;
    fan = 0;
    heater = 0;
    all_temp = 0;
    pos_setup = 0;
    tcal = 0;
}

void Xbaby::write_eeprom(int address, int number, float value_data){
    int first = 0;
    float last = 0;
    if(number == 1){
        EEPROM.write(address, value_data);
    }
    if(number == 2){
        first = value_data;
        last = (value_data - first)*10;
        EEPROM.write(address, first);
        EEPROM.write(address+1, last);
    }
}

int Xbaby::get_value_pos(int set_humidity, int value_humidity, int last_pos){
    if(pos_setup == 0){
        pos = last_pos;
        pos_setup = 1;
    }
    int err = set_humidity - value_humidity;
    if(err <= 0){
        if(millis() - tcal > 1){
            tcal = millis();
            pos++;
            if(pos >= 180){
                pos = 180;
                return pos;
            }
            return pos;
        }
    }

    if(err >= 0){
        if(millis() - tcal > 1){
            tcal = millis();
            pos--;
            if(pos <= 0){
                pos = 0;
                return pos;
            }
            return pos;
        }
    }
}

float Xbaby::get_value_baby_skin(float valueA, float valueB, int pin){
    float avr_temp = 0;
    avr_temp = pin;
        all_temp = ((-0.001*(pow(avr_temp, 2))) + (18*avr_temp) + 24125)/1000;   
        if(avr_temp <= 20){
            return all_temp = 0;
        }
        else{
        return all_temp;
     }
}
    


int Xbaby::get_value_fan(float set_temp, int mode, float temp_chamber, float temp_baby0, float temp_baby1, float set_baby_humidity, float baby_humidity){
    float err_chamber = (set_temp*10) - (temp_chamber*10);
    float err_baby0 = (set_temp*10) - (temp_baby0*10);
    float err_baby1 = (set_temp*10) - (temp_baby1*10);
    float err_humidity = (set_baby_humidity*10) - (baby_humidity*10);
    if(mode == 0){
        return 90;
    }
    if(mode == 1){
        if(temp_baby0 == 0 && temp_baby1 == 0){
            return 100;
        }else{
            if(temp_baby0 != 0){
                if(err_baby0 < -0.5){
                    if(err_humidity >= 0){
                        return 50;
                        //return 130;
                    }
                    return 100;
                    //return 155;
                }
                if(err_baby0 < 150 && err_baby0 >= 0.5){
                    if(err_humidity >= 0){
                        return 50;
                        //return 102;
                    }
                    return 130;
                    //return 155;
                }
                if(err_baby0 < 0.5 && err_baby0 >= -0.5){
                    if(err_humidity >= 0){
                        return 50;
                        //return 102;
                    }
                    return 100;
                    //return 155;
                }

            }
            if(temp_baby1 == 0 && temp_baby1 != 0){
                if(err_baby1 < -0.5){
                    if(err_humidity >= 0){
                        return 50;
                        //return 102;
                    }
                    return 100;
                    //return 155;
                }
                if(err_baby1 < 150 && err_baby1 >= 0.1){
                    if(err_humidity >= 0){
                        return 50;
                        //return 102;
                    }
                    return 130;
                    //return 102;
                }
                if(err_baby1 < 0.1 && err_baby1 >= -0.5){
                    if(err_humidity >= 0){
                        return 50;
                        //return 102;
                    }
                    return 100;
                    //return 155;
                }
            }
        }
    }
    
    if(mode == 2){
        if(temp_chamber < 0){
            return 100;
        }
        else{
            if(err_chamber < -0.5){
                if(err_humidity >= 0){
                    return 60;
                    //return 102;
                }
                return 100;
                //return 155;
            }
            if(err_chamber < 150 && err_chamber >= 0.1){
                if(err_humidity >= 0){
                    return 60;
                    //return 102;
                }
                return 130;
                //return 102;
            }
            if(err_chamber < 0.1 && err_chamber > -0.5){
                if(err_humidity >= 0){
                    return 60;
                    //return 102;
                }
                return 100;
                //return 155;
            }
        }
    }
}


int Xbaby::get_value_heater(float set_temp,int mode, int high, float temp_chamber, float temp_baby0, float temp_baby1){
    float err_chamber = (set_temp*10) - (temp_chamber*10); 
    float err_baby0 = (set_temp*10) - (temp_baby0*10);
    float err_baby1 = (set_temp*10) - (temp_baby1*10);
    if(mode == 1 && high == 0 ){
        if(temp_baby0 == 0 && temp_baby1 == 0){
            return 0;
        }        
        else{
            if(temp_baby0 != 0){
                if(err_baby0 < -0.1 && err_baby0 > -999){
                    return 15;
                }
                if(err_baby0 < 150 && err_baby0 >= 70.5){
                    return 220;
                }
                if(err_baby0 < 70.5 && err_baby0 >= 50.5){
                    return 200;
                }
                if(err_baby0 < 50.5 && err_baby0 >= 40.5){
                    return 170;
                }
                if(err_baby0 < 40.5 && err_baby0 >= 20.5){
                    return 150;
                }            
                if(err_baby0 < 20.5 && err_baby0 >= 10.5){
                    return 120;
                }
                if(err_baby0 < 10.5 && err_baby0 >= 5.5){
                    return 110;
                }
                if(err_baby0 < 5.5 && err_baby0 >= 1.5){
                    return 92;
                }
                if(err_baby0 < 1.5 && err_baby0 >= -0.1){
                    return 92;
                }
            }
        }
    }
    if(mode == 1 && high == 1){
        if(temp_baby0 == 0 && temp_baby1 == 0){
            return 0;
        }else{
             if(temp_baby0 != 0){
                if(err_baby0 < -0.1 && err_baby0 > -999){
                    return 30;
                }
                if(err_baby0 < 150 && err_baby0 >= 70.5){
                    return 225;
                }
                if(err_baby0 < 70.5 && err_baby0 >= 50.5){
                    return 220;
                }
                if(err_baby0 < 50.5 && err_baby0 >= 40.5){
                    return 210;
                }
                if(err_baby0 < 40.5 && err_baby0 >= 30.5){
                    return 180;
                }            
                if(err_baby0 < 30.5 && err_baby0 >= 10.5){
                    return 140;
                }
                if(err_baby0 < 10.5 && err_baby0 >= 5.5){
                    return 120;
                }
                if(err_baby0 < 5.5 && err_baby0 >= 1.5){
                    return 110;
                }
                if(err_baby0 < 1.5 && err_baby0 >= -0.1){
                    return 100 ;
                }                
            }
        }
    }    
    // }

    if(mode == 2 && high == 0){
        if(temp_chamber == 0){
            return 0;
        }
        if(set_temp <= 32.5){
                if(err_chamber < -0.5 && err_chamber > -999){
                    return 0;
                }
                if(err_chamber < 150 && err_chamber >= 70.5){
                    return 255;
                }
                if(err_chamber < 70.5 && err_chamber >= 50.5){
                    return 210;
                }
                if(err_chamber < 50.5 && err_chamber >= 30.5){
                    return 160;
                }
                if(err_chamber < 30.5 && err_chamber >= 20.5){
                    return 140;
                }            
                if(err_chamber < 20.5 && err_chamber >= 10.5){
                    return 100;
                }
                if(err_chamber < 10.5 && err_chamber >= 5.5){
                    return 80;
                }
                if(err_chamber < 5.5 && err_chamber >= 1.5){
                    return 60;
                }
                if(err_chamber < 1.5 && err_chamber >= -0.5){
                    return 40;
                }
        }
        if(set_temp > 32.5 && set_temp <= 34.5){
                if(err_chamber < -0.5 && err_chamber > -999){
                    return 10;
                }
                if(err_chamber < 150 && err_chamber >= 70.5){
                    return 255;
                }
                if(err_chamber < 70.5 && err_chamber >= 50.5){
                    return 210;
                }
                if(err_chamber < 50.5 && err_chamber >= 20.5){
                    return 160;
                }            
                if(err_chamber < 20.5 && err_chamber >= 10.5){
                    return 120;
                }
                if(err_chamber < 10.5 && err_chamber >= 5.5){
                    return 95;
                }
                if(err_chamber < 5.5 && err_chamber >= 1.5){
                    return 72;
                }
                if(err_chamber < 1.5 && err_chamber >= -0.5){
                    return 60;
                }
        }
       if(set_temp > 34.5){
                if(err_chamber < -0.5 && err_chamber > -999){
                    return 10;
                }
                if(err_chamber < 150 && err_chamber >= 70.5){
                    return 200;
                }
                if(err_chamber < 70.5 && err_chamber >= 50.5){
                    return 210;
                }
                if(err_chamber < 50.5 && err_chamber >= 20.5){
                    return 160;
                }            
                if(err_chamber < 20.5 && err_chamber >= 10.5){
                    return 150;
                }
                if(err_chamber < 10.5 && err_chamber >= 5.5){
                    return 120;
                }
                if(err_chamber < 5.5 && err_chamber >= 1.5){
                    return 120;
                }
                if(err_chamber < 1.5 && err_chamber >= 0){
                    return 100;
                }
                if(err_chamber < 0 && err_chamber >= -0.5){
                    return 90;
                }
        }        
    }
    if(mode == 2 && high == 1){
        if(temp_chamber == 0){
            return 0;
        }else{
            if(err_chamber < -0.5 && err_chamber > -999){
                return 20;
            } 
            if(err_chamber < 150 && err_chamber >= 70.5){
                return 255;
            }
            if(err_chamber < 70.5 && err_chamber >= 50.5){
                return 220;
            }
            if(err_chamber < 50.5 && err_chamber >= 30.5){
                return 200;
            }
            if(err_chamber < 30.5 && err_chamber >= 20.5){
                return 180;
            }            
            if(err_chamber < 20.5 && err_chamber >= 10.5){
                return 170;
            }
            if(err_chamber < 10.5 && err_chamber >= 5.5){
                return 150;
            }
            if(err_chamber < 5.5 && err_chamber >= 1.5){
                return 120;
            }
            if(err_chamber < 1.5 && err_chamber >= -0.5){
                return 114;
            }
        }
    }
}
