case 17: { //u32bool9
                static uint16_t msgid = 0;
                static uint8_t fnccode=0;
                ImGui::InputScalar("msgid10",ImGuiDataType_U16, &msgid);
                ImGui::InputScalar("fnccode10",ImGuiDataType_U8, &fnccode);

                ImGui::InputScalar("10u32",ImGuiDataType_U32, &command->u32bool9.Timeout);
                for (uint8_t i =0; i<13; i++) {
                    char temp[10] ={0,};
                    sprintf(temp, "bool%d",i);
                    ImGui::Checkbox(temp, &command->u32bool9.ChannelStatus[i]);
                }

                if(ImGui::Button("Generate CMD")) {
                    msgid = htons(msgid);
                    uint8_t stream[2] ={0,};
                    uint8_t sequence[2] = {0xC0,0x00};
                    uint8_t length[2] = {0x00, sizeof(u32bool9_t)};
                    memcpy(command->u32bool9.CmdHeader, &msgid, sizeof(uint16_t));
                    memcpy(command->u32bool9.CmdHeader + 2, sequence, sizeof(uint16_t));
                    memcpy(command->u32bool9.CmdHeader + 4, length, sizeof(uint16_t));
                    memcpy(command->u32bool9.CmdHeader + 7, &fnccode, sizeof(uint8_t));
                    pthread_join(p_thread[4], NULL);
                    packetsign * TestPacket = (packetsign *)malloc(2+2+4+sizeof(u32bool9_t)); // 8 is for data
                    TestPacket->Identifier = HVD_TEST;
                    TestPacket->PacketType = MIM_PT_TMTC_TEST;
                    TestPacket->Length = sizeof(u32bool9_t);
                    
                    uint16_t len = sizeof(u32bool9_t);
                    const uint8_t *byteptr = (uint8_t *)&command->u32bool9;
                    uint8_t checksum = 0xFF;
                    while (len--) {
                        checksum ^= *(byteptr++);
                    }
                    memcpy(command->u32bool9.CmdHeader+6, &checksum, sizeof(uint8_t));
                    memcpy(TestPacket->Data, &command->u32bool9, sizeof(u32bool9_t));
                    

                    //pthread_create(&p_thread[4], NULL, task_uplink_onorbit, (void *)TestPacket);
                }
                //ImGui::Text("0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x || %u %u %u %u %u %u %u",command->u32bool9.CmdHeader[0],
                //                command->u32bool9.CmdHeader[1], command->u32bool9.CmdHeader[2], command->u32bool9.CmdHeader[3], command->u32bool9.CmdHeader[4], command->u32bool9.CmdHeader[5], command->u32bool9.CmdHeader[6], command->u32bool9.CmdHeader[7],
                //                command->u32bool9.Timeout, command->u32bool9.ItemSize, command->u32bool9.ItemCount, command->u32bool9.Node, command->u32bool9.TableId, command->u32bool9.Address, command->u32bool9.Type);
                break;
            }