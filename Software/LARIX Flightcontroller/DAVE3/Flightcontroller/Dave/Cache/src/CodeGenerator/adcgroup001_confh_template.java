package CodeGenerator;

import java.util.*;
import com.ifx.davex.appjetinteract.App2JetInterface;

public class adcgroup001_confh_template
{
  protected static String nl;
  public static synchronized adcgroup001_confh_template create(String lineSeparator)
  {
    nl = lineSeparator;
    adcgroup001_confh_template result = new adcgroup001_confh_template();
    nl = null;
    return result;
  }

  public final String NL = nl == null ? (System.getProperties().getProperty("line.separator")) : nl;
  protected final String TEXT_1 = NL + "/*CODE_BLOCK_BEGIN[ADCGROUP001Conf.h]*/" + NL + "" + NL + "/******************************************************************************" + NL + " *" + NL + " * Copyright (C) 2013 Infineon Technologies AG. All rights reserved." + NL + " *" + NL + " * Infineon Technologies AG (Infineon) is supplying this software for use with" + NL + " * Infineon's microcontrollers." + NL + " * This file can be freely distributed within development tools that are" + NL + " * supporting such microcontrollers." + NL + " *" + NL + " * THIS SOFTWARE IS PROVIDED \"AS IS\".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED" + NL + " * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF" + NL + " * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE." + NL + " * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL," + NL + " * OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER." + NL + " *" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "**                                                                            **" + NL + "** PLATFORM : Infineon XMC4000/XMC1000 Series                                 **" + NL + "**                                                                            **" + NL + "** COMPILER : Compiler Independent                                            **" + NL + "**                                                                            **" + NL + "** AUTHOR   : DAVE App Developer                                              **" + NL + "**                                                                            **" + NL + "** MAY BE CHANGED BY USER [yes/Yes]: Yes                                      **" + NL + "**                                                                            **" + NL + "** MODIFICATION DATE : Jun 09, 2014                                          **" + NL + "**                                                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/*******************************************************************************" + NL + "**                       Author(s) Identity                                   **" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "** Initials     Name                                                          **" + NL + "** ---------------------------------------------------------------------------**" + NL + "** CM           DAVE App Developer                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/**" + NL + " * @file  ADCGROUP001_Conf.h" + NL + " *" + NL + " * @brief This file contains App parameter data as per GUI configurations" + NL + " *" + NL + " * Change History:" + NL + " * Date       version       Details" + NL + " * 01-Jan-2013   1.0.6    Initial code added for XMC1000" + NL + " * 29-Jan-2013   1.0.8    Merging with XMC1000 done" + NL + " * 09-Apr-2013   1.0.12   Channel selection style is removed and made EMXCSS=1" + NL + " *                        always" + NL + " * 24-Apr-2013   1.0.14   Ported to XMC4500 step AB " + NL + " * 11-Jun-2013   1.0.16   1.Variable name in manifest is corrected." + NL + " *                        2.In the Header comment section device name changed to" + NL + " *                         XMC1000." + NL + " *                        3.copy right year changed to 2013" + NL + " * 11-Sep-2013   1.0.20   1. Analog converter control(ANONC) Code sequence" + NL + " *                           corrected." + NL + " *                        2. Added support for the following devices under " + NL + " *                        XMC1200 and XMC1300 Series - XMC1201, XMC1202, XMC1301" + NL + " * 09-Jun-2014   1.0.28   ADC conversion time calculations are updated for XMC1X " + NL + " *                        devices." + NL + " *" + NL + " */" + NL + "#ifndef ADCGROUP001CONF_H_" + NL + "#define ADCGROUP001CONF_H_" + NL + "" + NL + "#ifdef __cplusplus" + NL + "extern \"C\" {" + NL + "#endif" + NL + "" + NL + "#include <DAVE3.h>" + NL;
  protected final String TEXT_2 = NL;
  protected final String TEXT_3 = NL + NL + NL + "extern const ADCGROUP001_HandleType ADCGROUP001_Handle";
  protected final String TEXT_4 = ";";
  protected final String TEXT_5 = NL + NL + "#ifdef __cplusplus" + NL + "}" + NL + "#endif" + NL + "  " + NL + "#endif /* ADCGROUP001CONF_H_ */" + NL + "" + NL + "/**" + NL + " * @}" + NL + " */" + NL + " " + NL + "/*CODE_BLOCK_END*/";
  protected final String TEXT_6 = NL;

  public String generate(Object argument)
  {
    final StringBuffer stringBuffer = new StringBuffer();
     App2JetInterface app = (App2JetInterface) argument; 
    stringBuffer.append(TEXT_1);
     String AppBaseuri = "app/adcgroup001/"; 
     String appInst  = null; 
    stringBuffer.append(TEXT_2);
     ArrayList<String> appsList = (ArrayList<String>)(app.getApps("app/adcgroup001/"));
for (String appIns : appsList ) {
    appInst = appIns.substring(appIns.lastIndexOf("/")+1);
    stringBuffer.append(TEXT_3);
    stringBuffer.append( appInst);
    stringBuffer.append(TEXT_4);
    }
    stringBuffer.append(TEXT_5);
    stringBuffer.append(TEXT_6);
    return stringBuffer.toString();
  }
}
