/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdint>
#include <string>
#ifndef __BLE_BUTTON_SERVICE_H__
#define __BLE_BUTTON_SERVICE_H__
using namespace std;

class ButtonService {
public:
    const static uint16_t BUTTON_SERVICE_UUID              = 0xA000;
    const static uint16_t BUTTON_STATE_CHARACTERISTIC_UUID = 0xA001;

    ButtonService(BLE &_ble, int buttonPressedInitial) :
        ble(_ble), buttonState(BUTTON_STATE_CHARACTERISTIC_UUID, &buttonPressedInitial, GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_BROADCAST)
    {
        //ID[0] = 'B'; ID[1] = '0'; ID[2] = '6'; ID[3] = '9'; ID[4] = '0'; ID[5] = '1'; ID[6] = '1'; ID[7] = '4'; ID[8] = '9'; ID[9] = '\0';
        GattCharacteristic *charTable[] = {&buttonState};
        //GattCharacteristic *charTable[] = {&ID};
        GattService         buttonService(ButtonService::BUTTON_SERVICE_UUID, charTable, sizeof(charTable) / sizeof(GattCharacteristic *));
        //GattService         buttonService(ButtonService::BUTTON_SERVICE_UUID, (&ID), sizeof(ID) / sizeof(GattCharacteristic *));
        ble.gattServer().addService(buttonService);
    }

    void updateButtonState(int newState) {
        ble.gattServer().write(buttonState.getValueHandle(), (uint8_t *)&newState, sizeof(int));
    }

private:
    BLE                              &ble;
    ReadOnlyGattCharacteristic<int>  buttonState;
    //ReadOnlyGattCharacteristic<*char>  ID;
};

#endif /* #ifndef __BLE_BUTTON_SERVICE_H__ */
