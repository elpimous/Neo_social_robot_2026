/*
 * Software License Agreement (GPLv2 License)
 *
 * Copyright (c) 2012 Thecorpora, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 * Authors: Miguel Angel Julian <miguel.a.j@openqbo.org>;
 *
 */

#include "qbo_arduqbo/drivers/arduqbo_instructions.h"

// TODO: Comentar el código

CComando::CComando(uint8_t number, int inputLength, int outputLength,
                   std::string inputType, std::string outputType)
: number_(number),
  inputLength_(inputLength),
  outputLength_(outputLength),
  inputType_(inputType),
  outputType_(outputType)
{}

int CComando::calcsize(const std::string &type)
{
    int size = 0;
    for (char c : type)
    {
        switch (c)
        {
        case 'b': size += 1; break;
        case 'h': size += 2; break;
        case 'f': size += 4; break;
        case 'l': size += 4; break;
        case 's': size += 1; break; // string char
        }
    }
    return size;
}


int CComando::serialize(std::vector<dataUnion> inputData, std::string &serializedData)
{
    if (inputLength_ != -1 && (int)inputData.size() != inputLength_)
        return -1;

    serializedData.clear();
    serializedData.push_back(number_);

    if (inputLength_ == 0)
    {
        serializedData.push_back(0x00);
    }
    else
    {
        int inputLength = inputLength_;
        if (inputLength == -1)
            inputLength = (int)inputData.size();

        std::string inputType;
        if (inputType_.size() == 1)
        {
            for (int i = 0; i < inputLength; i++)
                inputType += inputType_;
        }
        else
        {
            if (inputType_[0] != 'x' && (int)inputType_.size() != inputLength)
                return -1;
            std::string type;
            if (inputType_[0] != 'x')
            {
                type = inputType_;
            }
            else
            {
                std::string xType(inputType_);
                xType.erase(0, 1);
                for (int i = 0; i < (inputLength / calcsize(xType)); i++)
                    type += xType;
            }
            inputType = type;
        }

        if (inputLength != (int)inputData.size())
            return -1;

        std::string sd;
        uint8_t *data_p;

        for (int i = 0; i < inputLength; i++)
        {
            switch (inputType[i])
            {
            case 'b':
                sd.push_back(inputData[i].b);
                break;
            case 'h':
                data_p = (uint8_t *)(&inputData[i].h);
                sd.push_back(data_p[0]);
                sd.push_back(data_p[1]);
                break;
            case 'f':
                data_p = (uint8_t *)(&inputData[i].f);
                for (int j = 0; j < 4; ++j)
                    sd.push_back(data_p[j]);
                break;
            case 'l':
                data_p = (uint8_t *)(&inputData[i].l);
                for (int j = 0; j < 4; ++j)
                    sd.push_back(data_p[j]);
                break;
            case 's':
                sd += inputData[i].s;
                break;
            }
        }

        serializedData.push_back(sd.size());
        serializedData += sd;
    }

    // printf("🧾 serialize(): id=0x%02X, inputLength=%d, outputLength=%d, inputType=%s, outputType=%s\n",
    //        number_, inputLength_, outputLength_, inputType_.c_str(), outputType_.c_str());

    return 1;
}


// TODO: Revisar éste método que a veces da error
int CComando::deserialize(std::string inData, std::vector<dataUnion> &receivedData)
{
    receivedData.clear();

    if (outputLength_ == 0) {
        // printf("ℹ️ No response expected for command 0x%02X\n", number_);
        return 1;
    }

    if (inData.size() < 2)
        return -1;

    uint8_t cn = static_cast<uint8_t>(inData[0]);
    uint8_t idl = static_cast<uint8_t>(inData[1]);

    int outputLength = outputLength_;
    if (outputLength == -1)
        outputLength = idl / calcsize(outputType_);

    std::string typePattern;
    if (outputType_.size() == 1)
    {
        for (int i = 0; i < outputLength; i++)
            typePattern += outputType_;
    }
    else
    {
        if (outputType_[0] != 'x' && (int)outputType_.size() != outputLength)
            return -1;
        std::string type;
        if (outputType_[0] != 'x')
        {
            type = outputType_;
        }
        else
        {
            std::string xType(outputType_);
            xType.erase(0, 1);
            for (int i = 0; i < outputLength; i++)
                type += xType;
            outputLength = type.size();
        }
        typePattern = type;
    }

    // printf("🚨 deserialize : cn=0x%02X (expected 0x%02X), idl=%d (expected %d from outputType=%s)\n",
    //        cn, number_, idl, calcsize(typePattern), typePattern.c_str());

    if (cn != number_ || idl != calcsize(typePattern))
        return -1;

    if (outputLength == 0)
        return 1;

    dataUnion data;
    uint8_t readed = 2;
    uint8_t *data_p = (uint8_t *)(inData.c_str());

    for (int i = 0; i < outputLength; i++)
    {
        switch (typePattern[i])
        {
        case 'b':
            data.b = static_cast<uint8_t>(inData[readed]);
            readed += 1;
            break;
        case 'h':
            data.h = *((short *)(data_p + readed));
            readed += 2;
            break;
        case 'f':
            data.f = *((float *)(data_p + readed));
            readed += 4;
            break;
        case 'l':
            data.l = *((long *)(data_p + readed));
            readed += 4;
            break;
        case 's':
            data.s = std::string((char *)(data_p + readed));
            readed += data.s.size();
            break;
        }
        receivedData.push_back(data);
    }

    return 1;
}


