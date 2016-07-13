package CodeGenerator;

import java.util.*;
import com.ifx.davex.appjetinteract.App2JetInterface;

public class adc001_confh_template
{
  protected static String nl;
  public static synchronized adc001_confh_template create(String lineSeparator)
  {
    nl = lineSeparator;
    adc001_confh_template result = new adc001_confh_template();
    nl = null;
    return result;
  }

  public final String NL = nl == null ? (System.getProperties().getProperty("line.separator")) : nl;
  protected final String TEXT_1 = "/*CODE_BLOCK_BEGIN[ADC001Conf.h]*/" + NL + "" + NL + "/******************************************************************************" + NL + " *" + NL + " * Copyright (C) 2013 Infineon Technologies AG. All rights reserved." + NL + " *" + NL + " * Infineon Technologies AG (Infineon) is supplying this software for use with" + NL + " * Infineon's microcontrollers." + NL + " * This file can be freely distributed within development tools that are" + NL + " * supporting such microcontrollers." + NL + " *" + NL + " * THIS SOFTWARE IS PROVIDED \"AS IS\".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED" + NL + " * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF" + NL + " * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE." + NL + " * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL," + NL + " * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER." + NL + " *" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "**                                                                            **" + NL + "** PLATFORM : Infineon XMC4000/XMC1000 Series                                 **" + NL + "**                                                                            **" + NL + "** COMPILER : Compiler Independent                                            **" + NL + "**                                                                            **" + NL + "** AUTHOR   : DAVE App Developer                                              **" + NL + "**                                                                            **" + NL + "** MAY BE CHANGED BY USER [yes/Yes]: Yes                                      **" + NL + "**                                                                            **" + NL + "** MODIFICATION DATE : Sep 27, 2013                                           **" + NL + "**                                                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/*******************************************************************************" + NL + "**                       Author(s) Identity                                   **" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "** Initials     Name                                                          **" + NL + "** ---------------------------------------------------------------------------**" + NL + "** CM           DAVE App Developer                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/**" + NL + " * @file  ADC001_Conf.h" + NL + " *" + NL + " * @brief This file contains App parameter data as per GUI configurations" + NL + " *" + NL + " * Change History:" + NL + " *" + NL + " * Date       version       Details" + NL + " * 16-Dec-2012   1.0.4    Initial code added for XMC1000" + NL + " * 06-Mar-2013   1.0.6    App version changed for XMC1000 and XMC4400/XMC4200" + NL + " * 24-Apr-2013   1.0.8    Ported to XMC4500 step AB and" + NL + " *                        Source Result selection Feature added" + NL + " * 11-Jun-2013   1.0.10   1. In channel selection page Group C and Group D check " + NL + " *                        boxes made invisible for XMC1200/1300/4200 devices." + NL + " *                        2. In the Header comment section device name changed" + NL + " *                           to XMC1000." + NL + " *                        3. Tooltip for the Group channel selection is modified." + NL + " *                        4. New api ADC001_GetFastResult() implemented." + NL + " *                        5. Source Result register selection feature disabled" + NL + " *                           due to header file update not available.     " + NL + " * 27-Jun-2013   1.0.12   Source Result register selection feature removed to" + NL + " *                        simplify the app." + NL + " * 27-Sep-2013   1.0.16   Typos in API Documentation & .h comments is corrected. " + NL + " */" + NL + "#ifndef ADC001CONF_H_" + NL + "#define ADC001CONF_H_" + NL + "" + NL + "#ifdef __cplusplus" + NL + "extern \"C\" {" + NL + "#endif" + NL + "" + NL + "#include <DAVE3.h>" + NL;
  protected final String TEXT_2 = NL + NL + "extern const ADC001_HandleType ADC001_Handle0;" + NL + "extern const ADC001_ChannelHandleType* ADC001_ChannelHandlePtr[];" + NL;
  protected final String TEXT_3 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelAAHandle;";
  protected final String TEXT_4 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelABHandle;";
  protected final String TEXT_5 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelACHandle;";
  protected final String TEXT_6 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelADHandle;";
  protected final String TEXT_7 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelAEHandle;";
  protected final String TEXT_8 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelAFHandle;";
  protected final String TEXT_9 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelAGHandle;";
  protected final String TEXT_10 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelAHHandle;";
  protected final String TEXT_11 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBAHandle;";
  protected final String TEXT_12 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBBHandle;";
  protected final String TEXT_13 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBCHandle;";
  protected final String TEXT_14 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBDHandle;";
  protected final String TEXT_15 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBEHandle;";
  protected final String TEXT_16 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBFHandle;";
  protected final String TEXT_17 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBGHandle;";
  protected final String TEXT_18 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelBHHandle;";
  protected final String TEXT_19 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCAHandle;";
  protected final String TEXT_20 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCBHandle;";
  protected final String TEXT_21 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCCHandle;";
  protected final String TEXT_22 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCDHandle;";
  protected final String TEXT_23 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCEHandle;";
  protected final String TEXT_24 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCFHandle;";
  protected final String TEXT_25 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCGHandle;";
  protected final String TEXT_26 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelCHHandle;";
  protected final String TEXT_27 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDAHandle;";
  protected final String TEXT_28 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDBHandle;";
  protected final String TEXT_29 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDCHandle;";
  protected final String TEXT_30 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDDHandle;";
  protected final String TEXT_31 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDEHandle;";
  protected final String TEXT_32 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDFHandle;";
  protected final String TEXT_33 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDGHandle;";
  protected final String TEXT_34 = NL + "extern const ADC001_ChannelHandleType ADC001_ChannelDHHandle;";
  protected final String TEXT_35 = NL + NL + "#ifdef __cplusplus" + NL + "}" + NL + "#endif" + NL + "  " + NL + "#endif /* ADC001CONF_H_ */" + NL + "" + NL + "/**" + NL + " * @}" + NL + " */" + NL + " " + NL + "/*CODE_BLOCK_END*/";
  protected final String TEXT_36 = NL;

  public String generate(Object argument)
  {
    final StringBuffer stringBuffer = new StringBuffer();
     App2JetInterface app = (App2JetInterface) argument; 
    stringBuffer.append(TEXT_1);
     String AppBaseuri = "app/ADC001/"; 
     int GrpAChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupa"); 
     int GrpBChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupb"); 
     int GrpCChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupc"); 
     int GrpDChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupd"); 
    stringBuffer.append(TEXT_2);
     if (GrpAChannels > 0) { 
    stringBuffer.append(TEXT_3);
    }
     if (GrpAChannels > 1) { 
    stringBuffer.append(TEXT_4);
    }
     if (GrpAChannels > 2) { 
    stringBuffer.append(TEXT_5);
    }
     if (GrpAChannels > 3) { 
    stringBuffer.append(TEXT_6);
    }
     if (GrpAChannels > 4) { 
    stringBuffer.append(TEXT_7);
    }
     if (GrpAChannels > 5) { 
    stringBuffer.append(TEXT_8);
    }
     if (GrpAChannels > 6) { 
    stringBuffer.append(TEXT_9);
    }
     if (GrpAChannels > 7) { 
    stringBuffer.append(TEXT_10);
    }
     if (GrpBChannels > 0) { 
    stringBuffer.append(TEXT_11);
    }
     if (GrpBChannels > 1) { 
    stringBuffer.append(TEXT_12);
    }
     if (GrpBChannels > 2) { 
    stringBuffer.append(TEXT_13);
    }
     if (GrpBChannels > 3) { 
    stringBuffer.append(TEXT_14);
    }
     if (GrpBChannels > 4) { 
    stringBuffer.append(TEXT_15);
    }
     if (GrpBChannels > 5) { 
    stringBuffer.append(TEXT_16);
    }
     if (GrpBChannels > 6) { 
    stringBuffer.append(TEXT_17);
    }
     if (GrpBChannels > 7) { 
    stringBuffer.append(TEXT_18);
    }
     if (GrpCChannels > 0) { 
    stringBuffer.append(TEXT_19);
    }
     if (GrpCChannels > 1) { 
    stringBuffer.append(TEXT_20);
    }
     if (GrpCChannels > 2) { 
    stringBuffer.append(TEXT_21);
    }
     if (GrpCChannels > 3) { 
    stringBuffer.append(TEXT_22);
    }
     if (GrpCChannels > 4) { 
    stringBuffer.append(TEXT_23);
    }
     if (GrpCChannels > 5) { 
    stringBuffer.append(TEXT_24);
    }
     if (GrpCChannels > 6) { 
    stringBuffer.append(TEXT_25);
    }
     if (GrpCChannels > 7) { 
    stringBuffer.append(TEXT_26);
    }
     if (GrpDChannels > 0) { 
    stringBuffer.append(TEXT_27);
    }
     if (GrpDChannels > 1) { 
    stringBuffer.append(TEXT_28);
    }
     if (GrpDChannels > 2) { 
    stringBuffer.append(TEXT_29);
    }
     if (GrpDChannels > 3) { 
    stringBuffer.append(TEXT_30);
    }
     if (GrpDChannels > 4) { 
    stringBuffer.append(TEXT_31);
    }
     if (GrpDChannels > 5) { 
    stringBuffer.append(TEXT_32);
    }
     if (GrpDChannels > 6) { 
    stringBuffer.append(TEXT_33);
    }
     if (GrpDChannels > 7) { 
    stringBuffer.append(TEXT_34);
    }
    stringBuffer.append(TEXT_35);
    stringBuffer.append(TEXT_36);
    return stringBuffer.toString();
  }
}
