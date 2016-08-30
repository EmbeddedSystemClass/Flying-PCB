package CodeGenerator;

import java.util.*;
import java.text.*;
import com.ifx.davex.appjetinteract.App2JetInterface;

public class adc001_confc_template
{
  protected static String nl;
  public static synchronized adc001_confc_template create(String lineSeparator)
  {
    nl = lineSeparator;
    adc001_confc_template result = new adc001_confc_template();
    nl = null;
    return result;
  }

  public final String NL = nl == null ? (System.getProperties().getProperty("line.separator")) : nl;
  protected final String TEXT_1 = "/*CODE_BLOCK_BEGIN[ADC001_Conf.c]*/" + NL + "" + NL + "/*******************************************************************************" + NL + " Copyright (c) 2013, Infineon Technologies AG                                 **" + NL + " All rights reserved.                                                         **" + NL + "                                                                              **" + NL + " Redistribution and use in source and binary forms, with or without           **" + NL + " modification,are permitted provided that the following conditions are met:   **" + NL + "                                                                              **" + NL + " *Redistributions of source code must retain the above copyright notice,      **" + NL + " this list of conditions and the following disclaimer.                        **" + NL + " *Redistributions in binary form must reproduce the above copyright notice,   **" + NL + " this list of conditions and the following disclaimer in the documentation    **" + NL + " and/or other materials provided with the distribution.                       **" + NL + " *Neither the name of the copyright holders nor the names of its contributors **" + NL + " may be used to endorse or promote products derived from this software without**" + NL + " specific prior written permission.                                           **" + NL + "                                                                              **" + NL + " THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS \"AS IS\"  **" + NL + " AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE    **" + NL + " IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE   **" + NL + " ARE  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   **" + NL + " LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         **" + NL + " CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         **" + NL + " SUBSTITUTE GOODS OR  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    **" + NL + " INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN      **" + NL + " CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)       **" + NL + " ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE   **" + NL + " POSSIBILITY OF SUCH DAMAGE.                                                  **" + NL + "                                                                              **" + NL + " To improve the quality of the software, users are encouraged to share        **" + NL + " modifications, enhancements or bug fixes with Infineon Technologies AG       **" + NL + " dave@infineon.com).                                                          **" + NL + "                                                                              **" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "**                                                                            **" + NL + "** PLATFORM : Infineon XMC4000/XMC1000 Series                                 **" + NL + "**                                                                            **" + NL + "** COMPILER : Compiler Independent                                            **" + NL + "**                                                                            **" + NL + "** AUTHOR   : DAVE App Developer                                              **" + NL + "**                                                                            **" + NL + "** MAY BE CHANGED BY USER [yes/no]: Yes                                       **" + NL + "**                                                                            **" + NL + "** MODIFICATION DATE : Sep 27, 2013                                           **" + NL + "**                                                                            **" + NL + "*******************************************************************************/" + NL + "" + NL + "/*******************************************************************************" + NL + "**                       Author(s) Identity                                   **" + NL + "********************************************************************************" + NL + "**                                                                            **" + NL + "** Initials     Name                                                          **" + NL + "** ---------------------------------------------------------------------------**" + NL + "** CM           DAVE App Developer                                            **" + NL + "*******************************************************************************/";
  protected final String TEXT_2 = NL + "/**" + NL + " * @file  ADC001_Conf.c" + NL + " * @App Version ADC001 <";
  protected final String TEXT_3 = ">" + NL + " * @brief This file contains App parameter data as per GUI configurations" + NL + " *" + NL + " * Change History:" + NL + " *" + NL + " * Date       version       Details" + NL + " * 16-Dec-2012   1.0.4     Initial code added for XMC1000" + NL + " * 06-Mar-2013   1.0.6     App version changed for XMC1000 and XMC4400/XMC4200" + NL + " * 24-Apr-2013   1.0.8     Ported to XMC4500 step AB and " + NL + " *                         Source Result selection Feature added " + NL + " * 11-Jun-2013   1.0.10  1.In channel selection page Group C and Group D check" + NL + " *                         boxes made invisible for XMC1200/1300/4200 devices." + NL + " *                       2.In the Header comment section device name changed" + NL + " *                         to XMC1000." + NL + " *                       3.Tooltip for the Group channel selection is modified." + NL + " *                       4.New api ADC001_GetFastResult() implemented." + NL + " *                       5.Source Result register selection feature disabled" + NL + " *                         due to header file update not available.   " + NL + " * 27-Jun-2013   1.0.12    Source Result register selection feature removed to" + NL + " *                         simplify the app.   " + NL + " * 05-Sep-2013   1.0.14    Added support for the following devices under XMC1200" + NL + " *                         and XMC1300 Series XMC1201, XMC1202, XMC1301 " + NL + " * 27-Sep-13     1.0.16    Typos in API Documentation & .h comments is corrected. " + NL + " */" + NL + "" + NL + "#include <DAVE3.h>" + NL + NL;
  protected final String TEXT_4 = NL;
  protected final String TEXT_5 = NL + NL + "ADC001_DynamicDataType ADC001_DynamicHandle =" + NL + "{" + NL + "  .State = ADC001_UNINITIALIZED" + NL + "};" + NL;
  protected final String TEXT_6 = NL + "const ADC001_ChannelHandleType ADC001_ChannelAAHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_7 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_8 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_9 = NL + "};";
  protected final String TEXT_10 = NL;
  protected final String TEXT_11 = NL + "const ADC001_ChannelHandleType ADC001_ChannelABHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_12 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_13 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_14 = NL + "};";
  protected final String TEXT_15 = NL + "const ADC001_ChannelHandleType ADC001_ChannelACHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_16 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_17 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_18 = NL + "};";
  protected final String TEXT_19 = NL + NL + "const ADC001_ChannelHandleType ADC001_ChannelADHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_20 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_21 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_22 = NL + "};";
  protected final String TEXT_23 = NL + "const ADC001_ChannelHandleType ADC001_ChannelAEHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_24 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_25 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_26 = NL + "};";
  protected final String TEXT_27 = NL + "const ADC001_ChannelHandleType ADC001_ChannelAFHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_28 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_29 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_30 = NL + "};";
  protected final String TEXT_31 = NL + NL + "const ADC001_ChannelHandleType ADC001_ChannelAGHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_32 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_33 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_34 = NL + "};";
  protected final String TEXT_35 = NL + NL + "const ADC001_ChannelHandleType ADC001_ChannelAHHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_36 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_37 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_38 = NL + "};";
  protected final String TEXT_39 = NL + NL + "const ADC001_ChannelHandleType ADC001_ChannelBAHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_40 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_41 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_42 = NL + "};";
  protected final String TEXT_43 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBBHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_44 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_45 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_46 = NL + "};";
  protected final String TEXT_47 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBCHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_48 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_49 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_50 = NL + "};";
  protected final String TEXT_51 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBDHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_52 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_53 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_54 = NL + "};";
  protected final String TEXT_55 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBEHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_56 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_57 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_58 = NL + "};";
  protected final String TEXT_59 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBFHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_60 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_61 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_62 = NL + "};";
  protected final String TEXT_63 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBGHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_64 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_65 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_66 = NL + "};";
  protected final String TEXT_67 = NL + "const ADC001_ChannelHandleType ADC001_ChannelBHHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_68 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_69 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_70 = NL + "};";
  protected final String TEXT_71 = NL;
  protected final String TEXT_72 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCAHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_73 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_74 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_75 = NL + "};";
  protected final String TEXT_76 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCBHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_77 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_78 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_79 = NL + "};";
  protected final String TEXT_80 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCCHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_81 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_82 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_83 = NL + "};";
  protected final String TEXT_84 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCDHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_85 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_86 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_87 = NL + "};";
  protected final String TEXT_88 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCEHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_89 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_90 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_91 = NL + "};";
  protected final String TEXT_92 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCFHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_93 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_94 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_95 = NL + "};";
  protected final String TEXT_96 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCGHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_97 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_98 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_99 = NL + "};";
  protected final String TEXT_100 = NL + "const ADC001_ChannelHandleType ADC001_ChannelCHHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_101 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_102 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_103 = NL + "};";
  protected final String TEXT_104 = NL;
  protected final String TEXT_105 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDAHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_106 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_107 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_108 = NL + "};";
  protected final String TEXT_109 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDBHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_110 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_111 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_112 = NL + "};";
  protected final String TEXT_113 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDCHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_114 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_115 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_116 = NL + "};";
  protected final String TEXT_117 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDDHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_118 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_119 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_120 = NL + "};";
  protected final String TEXT_121 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDEHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_122 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_123 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_124 = NL + "};";
  protected final String TEXT_125 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDFHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_126 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_127 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_128 = NL + "};";
  protected final String TEXT_129 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDGHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_130 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_131 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_132 = NL + "};";
  protected final String TEXT_133 = NL + "const ADC001_ChannelHandleType ADC001_ChannelDHHandle = {" + NL + "  .kGroupNo = (uint8_t)";
  protected final String TEXT_134 = "," + NL + "  .kChannelNo = (uint8_t)";
  protected final String TEXT_135 = "," + NL + "  .kEnable = (uint8_t)";
  protected final String TEXT_136 = NL + "};";
  protected final String TEXT_137 = NL;
  protected final String TEXT_138 = NL + "const ADC001_ChannelHandleType* ADC001_ChannelHandlePtr[";
  protected final String TEXT_139 = "] = {";
  protected final String TEXT_140 = NL + "  &ADC001_ChannelAAHandle,";
  protected final String TEXT_141 = NL + "  &ADC001_ChannelABHandle,";
  protected final String TEXT_142 = NL + "  &ADC001_ChannelACHandle,";
  protected final String TEXT_143 = NL + "  &ADC001_ChannelADHandle,";
  protected final String TEXT_144 = NL + "  &ADC001_ChannelAEHandle,";
  protected final String TEXT_145 = NL + "  &ADC001_ChannelAFHandle,";
  protected final String TEXT_146 = NL + "  &ADC001_ChannelAGHandle,";
  protected final String TEXT_147 = NL + "  &ADC001_ChannelAHHandle,";
  protected final String TEXT_148 = NL + "  &ADC001_ChannelBAHandle,";
  protected final String TEXT_149 = NL + "  &ADC001_ChannelBBHandle,";
  protected final String TEXT_150 = NL + "  &ADC001_ChannelBCHandle,";
  protected final String TEXT_151 = NL + "  &ADC001_ChannelBDHandle,";
  protected final String TEXT_152 = NL + "  &ADC001_ChannelBEHandle,";
  protected final String TEXT_153 = NL + "  &ADC001_ChannelBFHandle,";
  protected final String TEXT_154 = NL;
  protected final String TEXT_155 = NL + "  &ADC001_ChannelBGHandle,";
  protected final String TEXT_156 = NL + "  &ADC001_ChannelBHHandle,";
  protected final String TEXT_157 = NL + "  &ADC001_ChannelCAHandle,";
  protected final String TEXT_158 = NL + "  &ADC001_ChannelCBHandle,";
  protected final String TEXT_159 = NL + "  &ADC001_ChannelCCHandle,";
  protected final String TEXT_160 = NL + "  &ADC001_ChannelCDHandle,";
  protected final String TEXT_161 = NL + "  &ADC001_ChannelCEHandle,";
  protected final String TEXT_162 = NL + "  &ADC001_ChannelCFHandle,";
  protected final String TEXT_163 = NL + "  &ADC001_ChannelCGHandle,";
  protected final String TEXT_164 = NL + "  &ADC001_ChannelCHHandle,";
  protected final String TEXT_165 = NL + "  &ADC001_ChannelDAHandle,";
  protected final String TEXT_166 = NL + "  &ADC001_ChannelDBHandle,";
  protected final String TEXT_167 = NL + "  &ADC001_ChannelDCHandle,";
  protected final String TEXT_168 = NL + "  &ADC001_ChannelDDHandle,";
  protected final String TEXT_169 = NL + "  &ADC001_ChannelDEHandle,";
  protected final String TEXT_170 = NL + "  &ADC001_ChannelDFHandle,";
  protected final String TEXT_171 = NL + "  &ADC001_ChannelDGHandle,";
  protected final String TEXT_172 = NL + "  &ADC001_ChannelDHHandle,";
  protected final String TEXT_173 = NL + "};";
  protected final String TEXT_174 = NL + NL + NL + "const ADC001_HandleType ADC001_Handle0 =" + NL + "{" + NL + "  .kChannels = (uint8_t)";
  protected final String TEXT_175 = "," + NL + "  .kGroup0ChSelect = (uint8_t)";
  protected final String TEXT_176 = ", " + NL + "  .kGroup1ChSelect = (uint8_t)";
  protected final String TEXT_177 = ",  ";
  protected final String TEXT_178 = NL + "  .kGroup2ChSelect = (uint8_t)";
  protected final String TEXT_179 = "," + NL + "  .kGroup3ChSelect = (uint8_t)";
  protected final String TEXT_180 = ",";
  protected final String TEXT_181 = NL + "  .kGrpAChannels = (uint8_t)";
  protected final String TEXT_182 = "," + NL + "  .kGrpBChannels = (uint8_t)";
  protected final String TEXT_183 = ",";
  protected final String TEXT_184 = NL + "  .kGrpCChannels = (uint8_t)";
  protected final String TEXT_185 = "," + NL + "  .kGrpDChannels = (uint8_t)";
  protected final String TEXT_186 = ",";
  protected final String TEXT_187 = NL + "  " + NL + "  .kActiveGroups = {(uint8_t)";
  protected final String TEXT_188 = ", (uint8_t)";
  protected final String TEXT_189 = " ";
  protected final String TEXT_190 = ", (uint8_t)";
  protected final String TEXT_191 = ", (uint8_t)";
  protected final String TEXT_192 = " ";
  protected final String TEXT_193 = "}, " + NL + "  " + NL + "  .kActiveChannels = {{(uint8_t)";
  protected final String TEXT_194 = ", (uint8_t)";
  protected final String TEXT_195 = ", (uint8_t)";
  protected final String TEXT_196 = ", (uint8_t)";
  protected final String TEXT_197 = ", (uint8_t)";
  protected final String TEXT_198 = ", (uint8_t)";
  protected final String TEXT_199 = ", (uint8_t)";
  protected final String TEXT_200 = ", (uint8_t)";
  protected final String TEXT_201 = " }," + NL + "                      {(uint8_t)";
  protected final String TEXT_202 = ", (uint8_t)";
  protected final String TEXT_203 = ", (uint8_t)";
  protected final String TEXT_204 = ", (uint8_t)";
  protected final String TEXT_205 = ", (uint8_t)";
  protected final String TEXT_206 = ", (uint8_t)";
  protected final String TEXT_207 = ", (uint8_t)";
  protected final String TEXT_208 = ", (uint8_t)";
  protected final String TEXT_209 = " } ";
  protected final String TEXT_210 = "," + NL + "                      {(uint8_t)";
  protected final String TEXT_211 = ", (uint8_t)";
  protected final String TEXT_212 = ", (uint8_t)";
  protected final String TEXT_213 = ", (uint8_t)";
  protected final String TEXT_214 = ", (uint8_t)";
  protected final String TEXT_215 = ", (uint8_t)";
  protected final String TEXT_216 = ", (uint8_t)";
  protected final String TEXT_217 = ", (uint8_t)";
  protected final String TEXT_218 = " }," + NL + "                      {(uint8_t)";
  protected final String TEXT_219 = ", (uint8_t)";
  protected final String TEXT_220 = ", (uint8_t)";
  protected final String TEXT_221 = ", (uint8_t)";
  protected final String TEXT_222 = ", (uint8_t)";
  protected final String TEXT_223 = ", (uint8_t)";
  protected final String TEXT_224 = ", (uint8_t)";
  protected final String TEXT_225 = ", (uint8_t)";
  protected final String TEXT_226 = " }";
  protected final String TEXT_227 = NL + "                     }, " + NL + "  " + NL + "  .kAsenEnable = (uint8_t)";
  protected final String TEXT_228 = "ADC001_ARBITRATION_SLOT_ENABLE,";
  protected final String TEXT_229 = "ADC001_ARBITRATION_SLOT_DISABLE,";
  protected final String TEXT_230 = NL + "  .kPriority = (uint8_t)ADC001_PRIORITY0,  " + NL + "  .kResultReadMode = (uint8_t)ADC001_WAIT_FOR_READ," + NL + "  .kStartmode = (uint8_t)ADC001_WAIT_FOR_START," + NL + "  .kTriggerMode = (uint8_t)";
  protected final String TEXT_231 = "ADC001_NOTRIGGER,";
  protected final String TEXT_232 = "ADC001_TRIGGER_UPON_FALLING_EDGE,";
  protected final String TEXT_233 = "ADC001_TRIGGER_UPON_RISING_EDGE,";
  protected final String TEXT_234 = "ADC001_TRIGGER_UPON_BOTH_EDGE,";
  protected final String TEXT_235 = NL + "  .kGatingMode = (uint8_t)";
  protected final String TEXT_236 = "ADC001_GATING_DISABLED,";
  protected final String TEXT_237 = "ADC001_GATING_ENABLED_ALWAYS,";
  protected final String TEXT_238 = "ADC001_GATING_ENABLED_FOR_HIGH,";
  protected final String TEXT_239 = "ADC001_GATING_ENABLED_FOR_LOW,";
  protected final String TEXT_240 = NL + "  .kTriggerEn = (uint8_t)";
  protected final String TEXT_241 = "1,";
  protected final String TEXT_242 = "0,";
  protected final String TEXT_243 = NL + "  .kAutoScan =   (uint8_t)";
  protected final String TEXT_244 = "ADC001_AUTOSCAN_ENABLE,";
  protected final String TEXT_245 = "ADC001_AUTOSCAN_DISABLE,";
  protected final String TEXT_246 = NL + "  .kResultControl = (uint8_t)";
  protected final String TEXT_247 = "ADC001_LEFT_ALIGN,";
  protected final String TEXT_248 = "ADC001_RIGHT_ALIGN,";
  protected final String TEXT_249 = NL + "  .kResultEvtEnable= (uint8_t)";
  protected final String TEXT_250 = "," + NL + "  .kResourceEvtEnable= (uint8_t)0U," + NL + "  .kIpClassSel = ";
  protected final String TEXT_251 = "(uint8_t)ADC001_GLOBAL_CLASS0";
  protected final String TEXT_252 = " (uint8_t)ADC001_GLOBAL_CLASS1 ";
  protected final String TEXT_253 = ",  " + NL + "  .VADCGlobalPtr= (VADC_GLOBAL_TypeDef*)VADC,  " + NL + "  .VADCGroup0Ptr= (VADC_G_TypeDef*)(void*) VADC_G0," + NL + "  .VADCGroup1Ptr= (VADC_G_TypeDef*)(void*) VADC_G1,";
  protected final String TEXT_254 = NL + "  .VADCGroup2Ptr= (VADC_G_TypeDef*)(void*) VADC_G2," + NL + "  .VADCGroup3Ptr= (VADC_G_TypeDef*)(void*) VADC_G3,";
  protected final String TEXT_255 = NL + "  .DynamicHandlePtr= &ADC001_DynamicHandle," + NL + " };" + NL + " " + NL + "/*CODE_BLOCK_END*/";
  protected final String TEXT_256 = NL;

  public String generate(Object argument)
  {
    final StringBuffer stringBuffer = new StringBuffer();
     App2JetInterface app = (App2JetInterface) argument; 
     int Is44Device = -1; 
     int Is42Device = -1; 
     int Is45Device = -1; 
     int Is1xDevice = -1; 
     Is44Device = ((app.getSoftwareId().substring(0,2).compareTo("44")==0)?1:0); 
     Is42Device = ((app.getSoftwareId().substring(0,2).compareTo("42")==0)?1:0); 
     Is45Device = ((app.getSoftwareId().substring(0,2).compareTo("45")==0)?1:0); 
     Is1xDevice = ((app.getSoftwareId().substring(0,1).compareTo("1")==0)?1:0); 
    stringBuffer.append(TEXT_1);
     String AppBaseuri = "app/ADC001/"; 
    stringBuffer.append(TEXT_2);
    stringBuffer.append(app.getAppVersion(AppBaseuri));
    stringBuffer.append(TEXT_3);
     String appInst  = null; 
     String BkgdUri = null; 
     int GrpAChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupa"); 
     int GrpBChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupb"); 
     int GrpCChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupc"); 
     int GrpDChannels = app.getIntegerValue(AppBaseuri + "0/adc001_irwnumberofchannelgroupd"); 
     int TotalChannels = GrpAChannels + GrpBChannels + GrpCChannels + GrpDChannels; 
     int ChEntry = 0; 
     String Group0AppUri = null; 
     String Group0Uri = null; 
     String Group0Inst = null; 
     String Group0No = null; 
     String Group1AppUri = null; 
     String Group1Uri = null; 
     String Group1Inst = null; 
     String Group1No = null; 
     String Group2AppUri = null; 
     String Group2Uri = null; 
     String Group2Inst = null; 
     String Group2No = null; 
     String Group3AppUri = null; 
     String Group3Uri = null; 
     String Group3Inst = null; 
     String Group3No = null; 
     String kernelUri = null; 
     String kernelNo = null; 
     int GroupA = 0; 
     int GroupB = 0; 
     int GroupC = 0; 
     int GroupD = 0; 
     String ChANum = null;
     String ChBNum = null;
     String ChCNum = null;
     String ChDNum = null;
     String ChENum = null;
     String ChFNum = null;
     String ChGNum = null;
     String ChHNum = null;
     String ChINum = null;
     String ChJNum = null;
     String ChKNum = null;
     String ChLNum = null;
     String ChMNum = null;
     String ChNNum = null;
     String ChONum = null;
     String ChPNum = null;
     String ChQNum = null;
     String ChRNum = null;
     String ChSNum = null;
     String ChTNum = null;
     String ChUNum = null;
     String ChVNum = null;
     String ChWNum = null;
     String ChXNum = null;
     String ChYNum = null;
     String ChZNum = null;
     String ChAANum = null;
     String ChABNum = null;
     String ChACNum = null;
     String ChADNum = null;
     String ChAENum = null;
     String ChAFNum = null;
     int g0asssel = 0; 
     int g1asssel = 0; 
     int g2asssel = 0; 
     int g3asssel = 0; 
     int g0chact = 0; 
     int g1chact = 0; 
     int g2chact = 0; 
     int g3chact = 0; 
     int g3groupact = 0; 
    stringBuffer.append(TEXT_4);
     BkgdUri = app.getMappedUri(AppBaseuri +  "0/backgnd" ); 
     Group0AppUri = app.getMappedUri(AppBaseuri +  "0/groupappa" ); 
     if ((Group0AppUri != null) && (Group0AppUri.trim() != "")) { 
     Group0Inst = Group0AppUri.substring(Group0AppUri.lastIndexOf("/") + 1);
     Group0Uri = app.getMappedUri(Group0AppUri +"/adcgroupconfig"); 
     if ((Group0Uri != null) && (Group0Uri.trim() != "")) { 
     Group0Uri = Group0Uri.substring(Group0Uri.length()-8, Group0Uri.indexOf("/config"));
     Group0No = Group0Uri.substring(Group0Uri.length() -1); 
     }
     }
     Group1AppUri = app.getMappedUri(AppBaseuri +  "0/groupappb" ); 
     if ((Group1AppUri != null) && (Group1AppUri.trim() != "")) { 
     Group1Inst = Group1AppUri.substring(Group1AppUri.lastIndexOf("/") + 1);
     Group1Uri = app.getMappedUri(Group1AppUri +"/adcgroupconfig"); 
     if ((Group1Uri != null) && (Group1Uri.trim() != "")) { 
     Group1Uri = Group1Uri.substring(Group1Uri.length()-8, Group1Uri.indexOf("/config"));
     Group1No = Group1Uri.substring(Group1Uri.length() -1); 
     }
     }
     Group2AppUri = app.getMappedUri(AppBaseuri +  "0/groupappc" ); 
     if ((Group2AppUri != null) && (Group2AppUri.trim() != "")) { 
     Group2Inst = Group2AppUri.substring(Group2AppUri.lastIndexOf("/") + 1);
     Group2Uri = app.getMappedUri(Group2AppUri +"/adcgroupconfig"); 
     if ((Group2Uri != null) && (Group2Uri.trim() != "")) { 
     Group2Uri = Group2Uri.substring(Group2Uri.length()-8, Group2Uri.indexOf("/config"));
     Group2No = Group2Uri.substring(Group2Uri.length() -1); 
     }
     }
     Group3AppUri = app.getMappedUri(AppBaseuri +  "0/groupappd" ); 
     if ((Group3AppUri != null) && (Group3AppUri.trim() != "")) { 
     Group3Inst = Group3AppUri.substring(Group3AppUri.lastIndexOf("/") + 1);
     Group3Uri = app.getMappedUri(Group3AppUri +"/adcgroupconfig"); 
     if ((Group3Uri != null) && (Group3Uri.trim() != "")) { 
     Group3Uri = Group3Uri.substring(Group3Uri.length()-8, Group3Uri.indexOf("/config"));
     Group3No = Group3Uri.substring(Group3Uri.length() -1); 
     }
     }
    stringBuffer.append(TEXT_5);
     if (GrpAChannels > 0) { 
     String ChAAppUri = null;
String ChAgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannela/0");
ChAAppUri = app.getMappedUri(AppBaseuri  +"0/channel_aa");
if ((ChAAppUri != null) && (ChAAppUri.trim() != "")) {
ChANum = ChAAppUri.substring(ChAAppUri.lastIndexOf("/")+1);
ChAgroup = ChAAppUri.substring(ChAAppUri.length()-6, ChAAppUri.indexOf("/ch/")); 
ChAgroup = ChAgroup.substring(ChAgroup.lastIndexOf("/") + 1 );
if(ChAgroup != null)
{
  GroupA = Integer.valueOf(ChAgroup);
  g3groupact |= (1 << GroupA); 
  if(Integer.valueOf(ChAgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChANum));
    g0chact |= (1 << Integer.valueOf(ChANum));
  }
  else if(Integer.valueOf(ChAgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChANum));
    g1chact |= (1 << Integer.valueOf(ChANum));
  }
  else if(Integer.valueOf(ChAgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChANum));
    g2chact |= (1 << Integer.valueOf(ChANum));
  }
  else if(Integer.valueOf(ChAgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChANum));
    g3chact |= (1 << Integer.valueOf(ChANum));
  }
}

    stringBuffer.append(TEXT_6);
    stringBuffer.append( ChAgroup);
    stringBuffer.append(TEXT_7);
    stringBuffer.append( ChANum);
    stringBuffer.append(TEXT_8);
    stringBuffer.append( chsel);
    stringBuffer.append(TEXT_9);
    }}
    stringBuffer.append(TEXT_10);
     if (GrpAChannels > 1) { 
     String ChBAppUri = null;
String ChBgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelb/0"); 
ChBAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ab");
if ((ChBAppUri != null) && (ChBAppUri.trim() != "")) {
ChBNum = ChBAppUri.substring(ChBAppUri.lastIndexOf("/")+1);
ChBgroup = ChBAppUri.substring(ChBAppUri.length()-6,ChBAppUri.indexOf("/ch/")); 
ChBgroup = ChBgroup.substring(ChBgroup.lastIndexOf("/") + 1 ); 
if(ChBgroup != null)
{
  if(Integer.valueOf(ChBgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChBNum));
    g0chact |= (1 << Integer.valueOf(ChBNum));
  }
  else if(Integer.valueOf(ChBgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChBNum));
    g1chact |= (1 << Integer.valueOf(ChBNum));
  }
  else if(Integer.valueOf(ChBgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChBNum));
    g2chact |= (1 << Integer.valueOf(ChBNum));
  }
  else if(Integer.valueOf(ChBgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChBNum));
    g3chact |= (1 << Integer.valueOf(ChBNum));
  }
}

    stringBuffer.append(TEXT_11);
    stringBuffer.append( ChBgroup);
    stringBuffer.append(TEXT_12);
    stringBuffer.append( ChBNum);
    stringBuffer.append(TEXT_13);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelb/0"));
    stringBuffer.append(TEXT_14);
    }}
     if (GrpAChannels > 2) { 
     String ChCAppUri = null;
String ChCgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelc/0");
ChCAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ac");
if ((ChCAppUri != null) && (ChCAppUri.trim() != "")) {
ChCNum = ChCAppUri.substring(ChCAppUri.lastIndexOf("/")+1);
ChCgroup = ChCAppUri.substring(ChCAppUri.length()-6,ChCAppUri.indexOf("/ch/")); 
ChCgroup = ChCgroup.substring(ChCgroup.lastIndexOf("/") + 1 ); 
if(ChCgroup != null)
{
  if(Integer.valueOf(ChCgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChCNum));
    g0chact |= (1 << Integer.valueOf(ChCNum));
  }
  else if(Integer.valueOf(ChCgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChCNum));
    g1chact |= (1 << Integer.valueOf(ChCNum));
  }
  else if(Integer.valueOf(ChCgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChCNum));
    g2chact |= (1 << Integer.valueOf(ChCNum));
  }
  else if(Integer.valueOf(ChCgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChCNum));
    g3chact |= (1 << Integer.valueOf(ChCNum));
  }
}

    stringBuffer.append(TEXT_15);
    stringBuffer.append( ChCgroup);
    stringBuffer.append(TEXT_16);
    stringBuffer.append( ChCNum);
    stringBuffer.append(TEXT_17);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelc/0"));
    stringBuffer.append(TEXT_18);
    }}
     if (GrpAChannels > 3) { 
     String ChDAppUri = null;
String ChDgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneld/0");
ChDAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ad");
if ((ChDAppUri != null) && (ChDAppUri.trim() != "")) {
ChDNum = ChDAppUri.substring(ChDAppUri.lastIndexOf("/")+1);
ChDgroup = ChDAppUri.substring(ChDAppUri.length()-6,ChDAppUri.indexOf("/ch/"));
ChDgroup = ChDgroup.substring(ChDgroup.lastIndexOf("/") + 1 ); 
if(ChDgroup != null)
{
  if(Integer.valueOf(ChDgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChDNum));
    g0chact |= (1 << Integer.valueOf(ChDNum));
  }
  else if(Integer.valueOf(ChDgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChDNum));
    g1chact |= (1 << Integer.valueOf(ChDNum));
  }
  else if(Integer.valueOf(ChDgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChDNum));
    g2chact |= (1 << Integer.valueOf(ChDNum));
  }
  else if(Integer.valueOf(ChDgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChDNum));
    g3chact |= (1 << Integer.valueOf(ChDNum));
  }
}

    stringBuffer.append(TEXT_19);
    stringBuffer.append( ChDgroup);
    stringBuffer.append(TEXT_20);
    stringBuffer.append( ChDNum);
    stringBuffer.append(TEXT_21);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneld/0"));
    stringBuffer.append(TEXT_22);
    }}
     if (GrpAChannels > 4) { 
     String ChEAppUri = null;
String ChEgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannele/0");
ChEAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ae");
if ((ChEAppUri != null) && (ChEAppUri.trim() != "")) {
ChENum = ChEAppUri.substring(ChEAppUri.lastIndexOf("/")+1);
ChEgroup = ChEAppUri.substring(ChEAppUri.length()-6,ChEAppUri.indexOf("/ch/")); 
ChEgroup = ChEgroup.substring(ChEgroup.lastIndexOf("/") + 1 ); 
if(ChEgroup != null)
{
  if(Integer.valueOf(ChEgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChENum));
    g0chact |= (1 << Integer.valueOf(ChENum));
  }
  else if(Integer.valueOf(ChEgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChENum));
    g1chact |= (1 << Integer.valueOf(ChENum));
  }
  else if(Integer.valueOf(ChEgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChENum));
    g2chact |= (1 << Integer.valueOf(ChENum));
  }
  else if(Integer.valueOf(ChEgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChENum));
    g3chact |= (1 << Integer.valueOf(ChENum));
  }
}

    stringBuffer.append(TEXT_23);
    stringBuffer.append( ChEgroup);
    stringBuffer.append(TEXT_24);
    stringBuffer.append( ChENum);
    stringBuffer.append(TEXT_25);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannele/0"));
    stringBuffer.append(TEXT_26);
    }}
     if (GrpAChannels > 5) { 
     String ChFAppUri = null;
String ChFgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelf/0");
ChFAppUri = app.getMappedUri(AppBaseuri  +"0/channel_af");
if ((ChFAppUri != null) && (ChFAppUri.trim() != "")) {
ChFNum = ChFAppUri.substring(ChFAppUri.lastIndexOf("/")+1);
ChFgroup = ChFAppUri.substring(ChFAppUri.length()-6,ChFAppUri.indexOf("/ch/")); 
ChFgroup = ChFgroup.substring(ChFgroup.lastIndexOf("/") + 1 );
if(ChFgroup != null)
{
  if(Integer.valueOf(ChFgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChFNum));
    g0chact |= (1 << Integer.valueOf(ChFNum));
  }
  else if(Integer.valueOf(ChFgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChFNum));
    g1chact |= (1 << Integer.valueOf(ChFNum));
  }
  else if(Integer.valueOf(ChFgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChFNum));
    g2chact |= (1 << Integer.valueOf(ChFNum));
  }
  else if(Integer.valueOf(ChFgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChFNum));
    g3chact |= (1 << Integer.valueOf(ChFNum));
  }
}

    stringBuffer.append(TEXT_27);
    stringBuffer.append( ChFgroup);
    stringBuffer.append(TEXT_28);
    stringBuffer.append( ChFNum);
    stringBuffer.append(TEXT_29);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelf/0"));
    stringBuffer.append(TEXT_30);
    }}
     if (GrpAChannels > 6) { 
     String ChGAppUri = null;
String ChGgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelg/0"); 
ChGAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ag");
if ((ChGAppUri != null) && (ChGAppUri.trim() != "")) {
ChGNum = ChGAppUri.substring(ChGAppUri.lastIndexOf("/")+1);
ChGgroup = ChGAppUri.substring(ChGAppUri.length()-6,ChGAppUri.indexOf("/ch/")); 
ChGgroup = ChGgroup.substring(ChGgroup.lastIndexOf("/") + 1 ); 
if(ChGgroup != null)
{
  if(Integer.valueOf(ChGgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChGNum));
    g0chact |= (1 << Integer.valueOf(ChGNum));
  }
  else if(Integer.valueOf(ChGgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChGNum));
    g1chact |= (1 << Integer.valueOf(ChGNum));
  }
  else if(Integer.valueOf(ChGgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChGNum));
    g2chact |= (1 << Integer.valueOf(ChGNum));
  }
  else if(Integer.valueOf(ChGgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChGNum));
    g3chact |= (1 << Integer.valueOf(ChGNum));
  }
}

    stringBuffer.append(TEXT_31);
    stringBuffer.append( ChGgroup);
    stringBuffer.append(TEXT_32);
    stringBuffer.append( ChGNum);
    stringBuffer.append(TEXT_33);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelg/0"));
    stringBuffer.append(TEXT_34);
    }}
     if (GrpAChannels > 7) { 
     String ChHAppUri = null;
String ChHgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelh/0"); 
ChHAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ah");
if ((ChHAppUri != null) && (ChHAppUri.trim() != "")) {
ChHNum = ChHAppUri.substring(ChHAppUri.lastIndexOf("/")+1);
ChHgroup = ChHAppUri.substring(ChHAppUri.length()-6,ChHAppUri.indexOf("/ch/")); 
ChHgroup = ChHgroup.substring(ChHgroup.lastIndexOf("/") + 1 ); 
if(ChHgroup != null)
{
  if(Integer.valueOf(ChHgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChHNum));
    g0chact |= (1 << Integer.valueOf(ChHNum));
  }
  else if(Integer.valueOf(ChHgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChHNum));
    g1chact |= (1 << Integer.valueOf(ChHNum));
  }
  else if(Integer.valueOf(ChHgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChHNum));
    g2chact |= (1 << Integer.valueOf(ChHNum));
  }
  else if(Integer.valueOf(ChHgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChHNum));
    g3chact |= (1 << Integer.valueOf(ChHNum));
  }
}

    stringBuffer.append(TEXT_35);
    stringBuffer.append( ChHgroup);
    stringBuffer.append(TEXT_36);
    stringBuffer.append( ChHNum);
    stringBuffer.append(TEXT_37);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelh/0"));
    stringBuffer.append(TEXT_38);
    }}
     if (GrpBChannels > 0) { 
     String ChIAppUri = null;
String ChIgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneli/0");
ChIAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ba");
if ((ChIAppUri != null) && (ChIAppUri.trim() != "")) {
ChINum = ChIAppUri.substring(ChIAppUri.lastIndexOf("/")+1);
ChIgroup = ChIAppUri.substring(ChIAppUri.length()-6,ChIAppUri.indexOf("/ch/")); 
ChIgroup = ChIgroup.substring(ChIgroup.lastIndexOf("/") + 1 ); 
if(ChIgroup != null)
{
  GroupB = Integer.valueOf(ChIgroup);
  g3groupact |= (1 << GroupB);
  if(Integer.valueOf(ChIgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChINum));
    g0chact |= (1 << Integer.valueOf(ChINum));
  }
  else if(Integer.valueOf(ChIgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChINum));
    g1chact |= (1 << Integer.valueOf(ChINum));
  }
  else if(Integer.valueOf(ChIgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChINum));
    g2chact |= (1 << Integer.valueOf(ChINum));
  }
  else if(Integer.valueOf(ChIgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChINum));
    g3chact |= (1 << Integer.valueOf(ChINum));
  }
}

    stringBuffer.append(TEXT_39);
    stringBuffer.append( ChIgroup);
    stringBuffer.append(TEXT_40);
    stringBuffer.append( ChINum);
    stringBuffer.append(TEXT_41);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneli/0"));
    stringBuffer.append(TEXT_42);
    }}
     if (GrpBChannels > 1) { 
     String ChJAppUri = null;
String ChJgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelj/0");
ChJAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bb");
if ((ChJAppUri != null) && (ChJAppUri.trim() != "")) {
ChJNum = ChJAppUri.substring(ChJAppUri.lastIndexOf("/")+1);
ChJgroup = ChJAppUri.substring(ChJAppUri.length()-6,ChJAppUri.indexOf("/ch/")); 
ChJgroup = ChJgroup.substring(ChJgroup.lastIndexOf("/") + 1 ); 
if(ChJgroup != null)
{
  if(Integer.valueOf(ChJgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChJNum));
    g0chact |= (1 << Integer.valueOf(ChJNum));
  }
  else if(Integer.valueOf(ChJgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChJNum));
    g1chact |= (1 << Integer.valueOf(ChJNum));
  }
  else if(Integer.valueOf(ChJgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChJNum));
    g2chact |= (1 << Integer.valueOf(ChJNum));
  }
  else if(Integer.valueOf(ChJgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChJNum));
    g3chact |= (1 << Integer.valueOf(ChJNum));
  }
}

    stringBuffer.append(TEXT_43);
    stringBuffer.append( ChJgroup);
    stringBuffer.append(TEXT_44);
    stringBuffer.append( ChJNum);
    stringBuffer.append(TEXT_45);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelj/0"));
    stringBuffer.append(TEXT_46);
    }}
     if (GrpBChannels > 2) { 
     String ChKAppUri = null;
String ChKgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelk/0");
ChKAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bc");
if ((ChKAppUri != null) && (ChKAppUri.trim() != "")) {
ChKNum = ChKAppUri.substring(ChKAppUri.lastIndexOf("/")+1);
ChKgroup = ChKAppUri.substring(ChKAppUri.length()-6,ChKAppUri.indexOf("/ch/")); 
ChKgroup = ChKgroup.substring(ChKgroup.lastIndexOf("/") + 1 ); 
if(ChKgroup != null)
{
  if(Integer.valueOf(ChKgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChKNum));
    g0chact |= (1 << Integer.valueOf(ChKNum));
  }
  else if(Integer.valueOf(ChKgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChKNum));
    g1chact |= (1 << Integer.valueOf(ChKNum));
  }
  else if(Integer.valueOf(ChKgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChKNum));
    g2chact |= (1 << Integer.valueOf(ChKNum));
  }
  else if(Integer.valueOf(ChKgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChKNum));
    g3chact |= (1 << Integer.valueOf(ChKNum));
  }
}

    stringBuffer.append(TEXT_47);
    stringBuffer.append( ChKgroup);
    stringBuffer.append(TEXT_48);
    stringBuffer.append( ChKNum);
    stringBuffer.append(TEXT_49);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelk/0"));
    stringBuffer.append(TEXT_50);
    }}
     if (GrpBChannels > 3) { 
     String ChLAppUri = null;
String ChLgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannell/0"); 
ChLAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bd");
if ((ChLAppUri != null) && (ChLAppUri.trim() != "")) {
ChLNum = ChLAppUri.substring(ChLAppUri.lastIndexOf("/")+1);
ChLgroup = ChLAppUri.substring(ChLAppUri.length()-6,ChLAppUri.indexOf("/ch/")); 
ChLgroup = ChLgroup.substring(ChLgroup.lastIndexOf("/") + 1 ); 
if(ChLgroup != null)
{
  if(Integer.valueOf(ChLgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChLNum));
    g0chact |= (1 << Integer.valueOf(ChLNum));
  }
  else if(Integer.valueOf(ChLgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChLNum));
    g1chact |= (1 << Integer.valueOf(ChLNum));
  }
  else if(Integer.valueOf(ChLgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChLNum));
    g2chact |= (1 << Integer.valueOf(ChLNum));
  }
  else if(Integer.valueOf(ChLgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChLNum));
    g3chact |= (1 << Integer.valueOf(ChLNum));
  }
}

    stringBuffer.append(TEXT_51);
    stringBuffer.append( ChLgroup);
    stringBuffer.append(TEXT_52);
    stringBuffer.append( ChLNum);
    stringBuffer.append(TEXT_53);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannell/0"));
    stringBuffer.append(TEXT_54);
    }}
     if (GrpBChannels > 4) { 
     String ChMAppUri = null;
String ChMgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelm/0"); 
ChMAppUri = app.getMappedUri(AppBaseuri  +"0/channel_be");
if ((ChMAppUri != null) && (ChMAppUri.trim() != "")) {
ChMNum = ChMAppUri.substring(ChMAppUri.lastIndexOf("/")+1);
ChMgroup = ChMAppUri.substring(ChMAppUri.length()-6,ChMAppUri.indexOf("/ch/"));
ChMgroup = ChMgroup.substring(ChMgroup.lastIndexOf("/") + 1 ); 
if(ChMgroup != null)
{
  if(Integer.valueOf(ChMgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChMNum));
    g0chact |= (1 << Integer.valueOf(ChMNum));
  }
  else if(Integer.valueOf(ChMgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChMNum));
    g1chact |= (1 << Integer.valueOf(ChMNum));
  }
  else if(Integer.valueOf(ChMgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChMNum));
    g2chact |= (1 << Integer.valueOf(ChMNum));
  }
  else if(Integer.valueOf(ChMgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChMNum));
    g3chact |= (1 << Integer.valueOf(ChMNum));
  }
}

    stringBuffer.append(TEXT_55);
    stringBuffer.append( ChMgroup);
    stringBuffer.append(TEXT_56);
    stringBuffer.append( ChMNum);
    stringBuffer.append(TEXT_57);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelm/0"));
    stringBuffer.append(TEXT_58);
    }}
     if (GrpBChannels > 5) { 
     String ChNAppUri = null;
String ChNgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneln/0");
ChNAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bf");
if ((ChNAppUri != null) && (ChNAppUri.trim() != "")) {
ChNNum = ChNAppUri.substring(ChNAppUri.lastIndexOf("/")+1);
ChNgroup = ChNAppUri.substring(ChNAppUri.length()-6,ChNAppUri.indexOf("/ch/")); 
ChNgroup = ChNgroup.substring(ChNgroup.lastIndexOf("/") + 1 ); 
if(ChNgroup != null)
{
  if(Integer.valueOf(ChNgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChNNum));
    g0chact |= (1 << Integer.valueOf(ChNNum));
  }
  else if(Integer.valueOf(ChNgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChNNum));
    g1chact |= (1 << Integer.valueOf(ChNNum));
  }
  else if(Integer.valueOf(ChNgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChNNum));
    g2chact |= (1 << Integer.valueOf(ChNNum));
  }
  else if(Integer.valueOf(ChNgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChNNum));
    g3chact |= (1 << Integer.valueOf(ChNNum));
  }
}

    stringBuffer.append(TEXT_59);
    stringBuffer.append( ChNgroup);
    stringBuffer.append(TEXT_60);
    stringBuffer.append( ChNNum);
    stringBuffer.append(TEXT_61);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchanneln/0"));
    stringBuffer.append(TEXT_62);
    }}
     if (GrpBChannels > 6) { 
     String ChOAppUri = null;
String ChOgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelo/0");
ChOAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bg");
if ((ChOAppUri != null) && (ChOAppUri.trim() != "")) {
ChONum = ChOAppUri.substring(ChOAppUri.lastIndexOf("/")+1);
ChOgroup = ChOAppUri.substring(ChOAppUri.length()-6,ChOAppUri.indexOf("/ch/"));
ChOgroup = ChOgroup.substring(ChOgroup.lastIndexOf("/") + 1 ); 
if(ChOgroup != null)
{
  if(Integer.valueOf(ChOgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChONum));
    g0chact |= (1 << Integer.valueOf(ChONum));
  }
  else if(Integer.valueOf(ChOgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChONum));
    g1chact |= (1 << Integer.valueOf(ChONum));
  }
  else if(Integer.valueOf(ChOgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChONum));
    g2chact |= (1 << Integer.valueOf(ChONum));
  }
  else if(Integer.valueOf(ChOgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChONum));
    g3chact |= (1 << Integer.valueOf(ChONum));
  }
}

    stringBuffer.append(TEXT_63);
    stringBuffer.append( ChOgroup);
    stringBuffer.append(TEXT_64);
    stringBuffer.append( ChONum);
    stringBuffer.append(TEXT_65);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelo/0"));
    stringBuffer.append(TEXT_66);
    }}
     if (GrpBChannels > 7) { 
     String ChPAppUri = null;
String ChPgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelp/0"); 
ChPAppUri = app.getMappedUri(AppBaseuri  +"0/channel_bh");
if ((ChPAppUri != null) && (ChPAppUri.trim() != "")) {
ChPNum = ChPAppUri.substring(ChPAppUri.lastIndexOf("/")+1);
ChPgroup = ChPAppUri.substring(ChPAppUri.length()-6,ChPAppUri.indexOf("/ch/")); 
ChPgroup = ChPgroup.substring(ChPgroup.lastIndexOf("/") + 1 );
if(ChPgroup != null)
{
  if(Integer.valueOf(ChPgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChPNum));
    g0chact |= (1 << Integer.valueOf(ChPNum));
  }
  else if(Integer.valueOf(ChPgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChPNum));
    g1chact |= (1 << Integer.valueOf(ChPNum));
  }
  else if(Integer.valueOf(ChPgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChPNum));
    g2chact |= (1 << Integer.valueOf(ChPNum));
  }
  else if(Integer.valueOf(ChPgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChPNum));
    g3chact |= (1 << Integer.valueOf(ChPNum));
  }
}

    stringBuffer.append(TEXT_67);
    stringBuffer.append( ChPgroup);
    stringBuffer.append(TEXT_68);
    stringBuffer.append( ChPNum);
    stringBuffer.append(TEXT_69);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelp/0"));
    stringBuffer.append(TEXT_70);
    }}
    stringBuffer.append(TEXT_71);
     if (GrpCChannels > 0) { 
     String ChQAppUri = null;
String ChQgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelq/0");
ChQAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ca");
if ((ChQAppUri != null) && (ChQAppUri.trim() != "")) {
ChQNum = ChQAppUri.substring(ChQAppUri.lastIndexOf("/")+1);
ChQgroup = ChQAppUri.substring(ChQAppUri.length()-6,ChQAppUri.indexOf("/ch/")); 
ChQgroup = ChQgroup.substring(ChQgroup.lastIndexOf("/") + 1 ); 
if(ChQgroup != null)
{
  GroupC = Integer.valueOf(ChQgroup);
  g3groupact |= (1 << GroupC);
  if(Integer.valueOf(ChQgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChQNum));
    g0chact |= (1 << Integer.valueOf(ChQNum));
  }
  else if(Integer.valueOf(ChQgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChQNum));
    g1chact |= (1 << Integer.valueOf(ChQNum));
  }
  else if(Integer.valueOf(ChQgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChQNum));
    g2chact |= (1 << Integer.valueOf(ChQNum));
  }
  else if(Integer.valueOf(ChQgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChQNum));
    g3chact |= (1 << Integer.valueOf(ChQNum));
  }
}

    stringBuffer.append(TEXT_72);
    stringBuffer.append( ChQgroup);
    stringBuffer.append(TEXT_73);
    stringBuffer.append( ChQNum);
    stringBuffer.append(TEXT_74);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelq/0"));
    stringBuffer.append(TEXT_75);
    }}
     if (GrpCChannels > 1) { 
     String ChRAppUri = null;
String ChRgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelr/0");
ChRAppUri = app.getMappedUri(AppBaseuri  +"0/channel_cb");
if ((ChRAppUri != null) && (ChRAppUri.trim() != "")) {
ChRNum = ChRAppUri.substring(ChRAppUri.lastIndexOf("/")+1);
ChRgroup = ChRAppUri.substring(ChRAppUri.length()-6,ChRAppUri.indexOf("/ch/")); 
ChRgroup = ChRgroup.substring(ChRgroup.lastIndexOf("/") + 1 ); 
if(ChRgroup != null)
{
  if(Integer.valueOf(ChRgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChRNum));
    g0chact |= (1 << Integer.valueOf(ChRNum));
  }
  else if(Integer.valueOf(ChRgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChRNum));
    g1chact |= (1 << Integer.valueOf(ChRNum));
  }
  else if(Integer.valueOf(ChRgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChRNum));
    g2chact |= (1 << Integer.valueOf(ChRNum));
  }
  else if(Integer.valueOf(ChRgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChRNum));
    g3chact |= (1 << Integer.valueOf(ChRNum));
  }
}

    stringBuffer.append(TEXT_76);
    stringBuffer.append( ChRgroup);
    stringBuffer.append(TEXT_77);
    stringBuffer.append( ChRNum);
    stringBuffer.append(TEXT_78);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelr/0"));
    stringBuffer.append(TEXT_79);
    }}
     if (GrpCChannels > 2) { 
     String ChSAppUri = null;
String ChSgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannels/0");
ChSAppUri = app.getMappedUri(AppBaseuri  +"0/channel_cc");
if ((ChSAppUri != null) && (ChSAppUri.trim() != "")) {
ChSNum = ChSAppUri.substring(ChSAppUri.lastIndexOf("/")+1);
ChSgroup = ChSAppUri.substring(ChSAppUri.length()-6,ChSAppUri.indexOf("/ch/")); 
ChSgroup = ChSgroup.substring(ChSgroup.lastIndexOf("/") + 1 ); 
if(ChSgroup != null)
{
  if(Integer.valueOf(ChSgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChSNum));
    g0chact |= (1 << Integer.valueOf(ChSNum));
  }
  else if(Integer.valueOf(ChSgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChSNum));
    g1chact |= (1 << Integer.valueOf(ChSNum));
  }
  else if(Integer.valueOf(ChSgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChSNum));
    g2chact |= (1 << Integer.valueOf(ChSNum));
  }
  else if(Integer.valueOf(ChSgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChSNum));
    g3chact |= (1 << Integer.valueOf(ChSNum));
  }
}

    stringBuffer.append(TEXT_80);
    stringBuffer.append( ChSgroup);
    stringBuffer.append(TEXT_81);
    stringBuffer.append( ChSNum);
    stringBuffer.append(TEXT_82);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannels/0"));
    stringBuffer.append(TEXT_83);
    }}
     if (GrpCChannels > 3) { 
     String ChTAppUri = null;
String ChTgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelt/0");
ChTAppUri = app.getMappedUri(AppBaseuri  +"0/channel_cd");
if ((ChTAppUri != null) && (ChTAppUri.trim() != "")) {
ChTNum = ChTAppUri.substring(ChTAppUri.lastIndexOf("/")+1);
ChTgroup = ChTAppUri.substring(ChTAppUri.length()-6,ChTAppUri.indexOf("/ch/")); 
ChTgroup = ChTgroup.substring(ChTgroup.lastIndexOf("/") + 1 ); 
if(ChTgroup != null)
{
  if(Integer.valueOf(ChTgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChTNum));
    g0chact |= (1 << Integer.valueOf(ChTNum));
  }
  else if(Integer.valueOf(ChTgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChTNum));
    g1chact |= (1 << Integer.valueOf(ChTNum));
  }
  else if(Integer.valueOf(ChTgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChTNum));
    g2chact |= (1 << Integer.valueOf(ChTNum));
  }
  else if(Integer.valueOf(ChTgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChTNum));
    g3chact |= (1 << Integer.valueOf(ChTNum));
  }
}

    stringBuffer.append(TEXT_84);
    stringBuffer.append( ChTgroup);
    stringBuffer.append(TEXT_85);
    stringBuffer.append( ChTNum);
    stringBuffer.append(TEXT_86);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelt/0"));
    stringBuffer.append(TEXT_87);
    }}
     if (GrpCChannels > 4) { 
     String ChUAppUri = null;
String ChUgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelu/0");
ChUAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ce");
if ((ChUAppUri != null) && (ChUAppUri.trim() != "")) {
ChUNum = ChUAppUri.substring(ChUAppUri.lastIndexOf("/")+1);
ChUgroup = ChUAppUri.substring(ChUAppUri.length()-6,ChUAppUri.indexOf("/ch/")); 
ChUgroup = ChUgroup.substring(ChUgroup.lastIndexOf("/") + 1 ); 
if(ChUgroup != null)
{
  if(Integer.valueOf(ChUgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChUNum));
    g0chact |= (1 << Integer.valueOf(ChUNum));
  }
  else if(Integer.valueOf(ChUgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChUNum));
    g1chact |= (1 << Integer.valueOf(ChUNum));
  }
  else if(Integer.valueOf(ChUgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChUNum));
    g2chact |= (1 << Integer.valueOf(ChUNum));
  }
  else if(Integer.valueOf(ChUgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChUNum));
    g3chact |= (1 << Integer.valueOf(ChUNum));
  }
}

    stringBuffer.append(TEXT_88);
    stringBuffer.append( ChUgroup);
    stringBuffer.append(TEXT_89);
    stringBuffer.append( ChUNum);
    stringBuffer.append(TEXT_90);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelu/0"));
    stringBuffer.append(TEXT_91);
    }}
     if (GrpCChannels > 5) { 
     String ChVAppUri = null;
String ChVgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelv/0");
ChVAppUri = app.getMappedUri(AppBaseuri  +"0/channel_cf");
if ((ChVAppUri != null) && (ChVAppUri.trim() != "")) {
ChVNum = ChVAppUri.substring(ChVAppUri.lastIndexOf("/")+1);
ChVgroup = ChVAppUri.substring(ChVAppUri.length()-6,ChVAppUri.indexOf("/ch/")); 
ChVgroup = ChVgroup.substring(ChVgroup.lastIndexOf("/") + 1 ); 
if(ChVgroup != null)
{
  if(Integer.valueOf(ChVgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChVNum));
    g0chact |= (1 << Integer.valueOf(ChVNum));
  }
  else if(Integer.valueOf(ChVgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChVNum));
    g1chact |= (1 << Integer.valueOf(ChVNum));
  }
  else if(Integer.valueOf(ChVgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChVNum));
    g2chact |= (1 << Integer.valueOf(ChVNum));
  }
  else if(Integer.valueOf(ChVgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChVNum));
    g3chact |= (1 << Integer.valueOf(ChVNum));
  }
}

    stringBuffer.append(TEXT_92);
    stringBuffer.append( ChVgroup);
    stringBuffer.append(TEXT_93);
    stringBuffer.append( ChVNum);
    stringBuffer.append(TEXT_94);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelv/0"));
    stringBuffer.append(TEXT_95);
    }}
     if (GrpCChannels > 6) { 
     String ChWAppUri = null;
String ChWgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelw/0");
ChWAppUri = app.getMappedUri(AppBaseuri  +"0/channel_cg");
if ((ChWAppUri != null) && (ChWAppUri.trim() != "")) {
ChWNum = ChWAppUri.substring(ChWAppUri.lastIndexOf("/")+1);
ChWgroup = ChWAppUri.substring(ChWAppUri.length()-6,ChWAppUri.indexOf("/ch/"));
ChWgroup = ChWgroup.substring(ChWgroup.lastIndexOf("/") + 1 ); 
if(ChWgroup != null)
{
  if(Integer.valueOf(ChWgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChWNum));
    g0chact |= (1 << Integer.valueOf(ChWNum));
  }
  else if(Integer.valueOf(ChWgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChWNum));
    g1chact |= (1 << Integer.valueOf(ChWNum));
  }
  else if(Integer.valueOf(ChWgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChWNum));
    g2chact |= (1 << Integer.valueOf(ChWNum));
  }
  else if(Integer.valueOf(ChWgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChWNum));
    g3chact |= (1 << Integer.valueOf(ChWNum));
  }
}

    stringBuffer.append(TEXT_96);
    stringBuffer.append( ChWgroup);
    stringBuffer.append(TEXT_97);
    stringBuffer.append( ChWNum);
    stringBuffer.append(TEXT_98);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelw/0"));
    stringBuffer.append(TEXT_99);
    }}
     if (GrpCChannels > 7) { 
     String ChXAppUri = null;
String ChXgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelx/0");
ChXAppUri = app.getMappedUri(AppBaseuri  +"0/channel_ch");
if ((ChXAppUri != null) && (ChXAppUri.trim() != "")) {
ChXNum = ChXAppUri.substring(ChXAppUri.lastIndexOf("/")+1);
ChXgroup = ChXAppUri.substring(ChXAppUri.length()-6,ChXAppUri.indexOf("/ch/")); 
ChXgroup = ChXgroup.substring(ChXgroup.lastIndexOf("/") + 1 ); 
if(ChXgroup != null)
{
  if(Integer.valueOf(ChXgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChXNum));
    g0chact |= (1 << Integer.valueOf(ChXNum));
  }
  else if(Integer.valueOf(ChXgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChXNum));
    g1chact |= (1 << Integer.valueOf(ChXNum));
  }
  else if(Integer.valueOf(ChXgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChXNum));
    g2chact |= (1 << Integer.valueOf(ChXNum));
  }
  else if(Integer.valueOf(ChXgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChXNum));
    g3chact |= (1 << Integer.valueOf(ChXNum));
  }
}

    stringBuffer.append(TEXT_100);
    stringBuffer.append( ChXgroup);
    stringBuffer.append(TEXT_101);
    stringBuffer.append( ChXNum);
    stringBuffer.append(TEXT_102);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelx/0"));
    stringBuffer.append(TEXT_103);
    }}
    stringBuffer.append(TEXT_104);
     if (GrpDChannels > 0) { 
     String ChYAppUri = null;
String ChYgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannely/0");
ChYAppUri = app.getMappedUri(AppBaseuri  +"0/channel_da");
if ((ChYAppUri != null) && (ChYAppUri.trim() != "")) {
ChYNum = ChYAppUri.substring(ChYAppUri.lastIndexOf("/")+1);
ChYgroup = ChYAppUri.substring(ChYAppUri.length()-6,ChYAppUri.indexOf("/ch/")); 
ChYgroup = ChYgroup.substring(ChYgroup.lastIndexOf("/") + 1 );
if(ChYgroup != null)
{
  GroupD = Integer.valueOf(ChYgroup);
  g3groupact |= (1 << GroupD);
  if(Integer.valueOf(ChYgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChYNum));
    g0chact |= (1 << Integer.valueOf(ChYNum));
  }
  else if(Integer.valueOf(ChYgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChYNum));
    g1chact |= (1 << Integer.valueOf(ChYNum));
  }
  else if(Integer.valueOf(ChYgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChYNum));
    g2chact |= (1 << Integer.valueOf(ChYNum));
  }
  else if(Integer.valueOf(ChYgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChYNum));
    g3chact |= (1 << Integer.valueOf(ChYNum));
  }
}

    stringBuffer.append(TEXT_105);
    stringBuffer.append( ChYgroup);
    stringBuffer.append(TEXT_106);
    stringBuffer.append( ChYNum);
    stringBuffer.append(TEXT_107);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannely/0"));
    stringBuffer.append(TEXT_108);
    }}
     if (GrpDChannels > 1) { 
     String ChZAppUri = null;
String ChZgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelz/0");
ChZAppUri = app.getMappedUri(AppBaseuri  +"0/channel_db");
if ((ChZAppUri != null) && (ChZAppUri.trim() != "")) {
ChZNum = ChZAppUri.substring(ChZAppUri.lastIndexOf("/")+1);
ChZgroup = ChZAppUri.substring(ChZAppUri.length()-6,ChZAppUri.indexOf("/ch/")); 
ChZgroup = ChZgroup.substring(ChZgroup.lastIndexOf("/") + 1 ); 
if(ChZgroup != null)
{
  if(Integer.valueOf(ChZgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChZNum));
    g0chact |= (1 << Integer.valueOf(ChZNum));
  }
  else if(Integer.valueOf(ChZgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChZNum));
    g1chact |= (1 << Integer.valueOf(ChZNum));
  }
  else if(Integer.valueOf(ChZgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChZNum));
    g2chact |= (1 << Integer.valueOf(ChZNum));
  }
  else if(Integer.valueOf(ChZgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChZNum));
    g3chact |= (1 << Integer.valueOf(ChZNum));
  }
}

    stringBuffer.append(TEXT_109);
    stringBuffer.append( ChZgroup);
    stringBuffer.append(TEXT_110);
    stringBuffer.append( ChZNum);
    stringBuffer.append(TEXT_111);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelz/0"));
    stringBuffer.append(TEXT_112);
    }}
     if (GrpDChannels > 2) { 
     String ChAAAppUri = null;
String ChAAgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelaa/0");
ChAAAppUri = app.getMappedUri(AppBaseuri  +"0/channel_dc");
if ((ChAAAppUri != null) && (ChAAAppUri.trim() != "")) {
ChAANum = ChAAAppUri.substring(ChAAAppUri.lastIndexOf("/")+1);
ChAAgroup = ChAAAppUri.substring(ChAAAppUri.length()-6,ChAAAppUri.indexOf("/ch/")); 
ChAAgroup = ChAAgroup.substring(ChAAgroup.lastIndexOf("/") + 1 ); 
if(ChAAgroup != null)
{
  if(Integer.valueOf(ChAAgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChAANum));
    g0chact |= (1 << Integer.valueOf(ChAANum));
  }
  else if(Integer.valueOf(ChAAgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChAANum));
    g1chact |= (1 << Integer.valueOf(ChAANum));
  }
  else if(Integer.valueOf(ChAAgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChAANum));
    g2chact |= (1 << Integer.valueOf(ChAANum));
  }
  else if(Integer.valueOf(ChAAgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChAANum));
    g3chact |= (1 << Integer.valueOf(ChAANum));
  }
}

    stringBuffer.append(TEXT_113);
    stringBuffer.append( ChAAgroup);
    stringBuffer.append(TEXT_114);
    stringBuffer.append( ChAANum);
    stringBuffer.append(TEXT_115);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelaa/0"));
    stringBuffer.append(TEXT_116);
    }}
     if (GrpDChannels > 3) { 
     String ChABAppUri = null;
String ChABgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelab/0");
ChABAppUri = app.getMappedUri(AppBaseuri  +"0/channel_dd");
if ((ChABAppUri != null) && (ChABAppUri.trim() != "")) {
ChABNum = ChABAppUri.substring(ChABAppUri.lastIndexOf("/")+1);
ChABgroup = ChABAppUri.substring(ChABAppUri.length()-6,ChABAppUri.indexOf("/ch/")); 
ChABgroup = ChABgroup.substring(ChABgroup.lastIndexOf("/") + 1 ); 
if(ChABgroup != null)
{
  if(Integer.valueOf(ChABgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChABNum));
    g0chact |= (1 << Integer.valueOf(ChABNum));
  }
  else if(Integer.valueOf(ChABgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChABNum));
    g1chact |= (1 << Integer.valueOf(ChABNum));
  }
  else if(Integer.valueOf(ChABgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChABNum));
    g2chact |= (1 << Integer.valueOf(ChABNum));
  }
  else if(Integer.valueOf(ChABgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChABNum));
    g3chact |= (1 << Integer.valueOf(ChABNum));
  }
}

    stringBuffer.append(TEXT_117);
    stringBuffer.append( ChABgroup);
    stringBuffer.append(TEXT_118);
    stringBuffer.append( ChABNum);
    stringBuffer.append(TEXT_119);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelab/0"));
    stringBuffer.append(TEXT_120);
    }}
     if (GrpDChannels > 4) { 
     String ChACAppUri = null;
String ChACgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelac/0");
ChACAppUri = app.getMappedUri(AppBaseuri  +"0/channel_de");
if ((ChACAppUri != null) && (ChACAppUri.trim() != "")) {
ChACNum = ChACAppUri.substring(ChACAppUri.lastIndexOf("/")+1);
ChACgroup = ChACAppUri.substring(ChACAppUri.length()-6,ChACAppUri.indexOf("/ch/")); 
ChACgroup = ChACgroup.substring(ChACgroup.lastIndexOf("/") + 1 ); 
if(ChACgroup != null)
{
  if(Integer.valueOf(ChACgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChACNum));
    g0chact |= (1 << Integer.valueOf(ChACNum));
  }
  else if(Integer.valueOf(ChACgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChACNum));
    g1chact |= (1 << Integer.valueOf(ChACNum));
  }
  else if(Integer.valueOf(ChACgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChACNum));
    g2chact |= (1 << Integer.valueOf(ChACNum));
  }
  else if(Integer.valueOf(ChACgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChACNum));
    g3chact |= (1 << Integer.valueOf(ChACNum));
  }
}

    stringBuffer.append(TEXT_121);
    stringBuffer.append( ChACgroup);
    stringBuffer.append(TEXT_122);
    stringBuffer.append( ChACNum);
    stringBuffer.append(TEXT_123);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelac/0"));
    stringBuffer.append(TEXT_124);
    }}
     if (GrpDChannels > 5) { 
     String ChADAppUri = null;
String ChADgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelad/0");
ChADAppUri = app.getMappedUri(AppBaseuri  +"0/channel_df");
if ((ChADAppUri != null) && (ChADAppUri.trim() != "")) {
ChADNum = ChADAppUri.substring(ChADAppUri.lastIndexOf("/")+1);
ChADgroup = ChADAppUri.substring(ChADAppUri.length()-6,ChADAppUri.indexOf("/ch/")); 
ChADgroup = ChADgroup.substring(ChADgroup.lastIndexOf("/") + 1 ); 
if(ChADgroup != null)
{
  if(Integer.valueOf(ChADgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChADNum));
    g0chact |= (1 << Integer.valueOf(ChADNum));
  }
  else if(Integer.valueOf(ChADgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChADNum));
    g1chact |= (1 << Integer.valueOf(ChADNum));
  }
  else if(Integer.valueOf(ChADgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChADNum));
    g2chact |= (1 << Integer.valueOf(ChADNum));
  }
  else if(Integer.valueOf(ChADgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChADNum));
    g3chact |= (1 << Integer.valueOf(ChADNum));
  }
}

    stringBuffer.append(TEXT_125);
    stringBuffer.append( ChADgroup);
    stringBuffer.append(TEXT_126);
    stringBuffer.append( ChADNum);
    stringBuffer.append(TEXT_127);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelad/0"));
    stringBuffer.append(TEXT_128);
    }}
     if (GrpDChannels > 6) { 
     String ChAEAppUri = null;
String ChAEgroup = null; 
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelae/0");
ChAEAppUri = app.getMappedUri(AppBaseuri  +"0/channel_dg");
if ((ChAEAppUri != null) && (ChAEAppUri.trim() != "")) {
ChAENum = ChAEAppUri.substring(ChAEAppUri.lastIndexOf("/")+1);
ChAEgroup = ChAEAppUri.substring(ChAEAppUri.length()-6,ChAEAppUri.indexOf("/ch/")); 
ChAEgroup = ChAEgroup.substring(ChAEgroup.lastIndexOf("/") + 1 ); 
if(ChAEgroup != null)
{
  if(Integer.valueOf(ChAEgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChAENum));
    g0chact |= (1 << Integer.valueOf(ChAENum));
  }
  else if(Integer.valueOf(ChAEgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChAENum));
    g1chact |= (1 << Integer.valueOf(ChAENum));
  }
  else if(Integer.valueOf(ChAEgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChAENum));
    g2chact |= (1 << Integer.valueOf(ChAENum));
  }
  else if(Integer.valueOf(ChAEgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChAENum));
    g3chact |= (1 << Integer.valueOf(ChAENum));
  }
}

    stringBuffer.append(TEXT_129);
    stringBuffer.append( ChAEgroup);
    stringBuffer.append(TEXT_130);
    stringBuffer.append( ChAENum);
    stringBuffer.append(TEXT_131);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelae/0"));
    stringBuffer.append(TEXT_132);
    }}
     if (GrpDChannels > 7) { 
     String ChAFAppUri = null;
String ChAFgroup = null;
int chsel = app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelaf/0"); 
ChAFAppUri = app.getMappedUri(AppBaseuri  +"0/channel_dh");
if ((ChAFAppUri != null) && (ChAFAppUri.trim() != "")) {
ChAFNum = ChAFAppUri.substring(ChAFAppUri.lastIndexOf("/")+1);
ChAFgroup = ChAFAppUri.substring(ChAFAppUri.length()-6,ChAFAppUri.indexOf("/ch/")); 
ChAFgroup = ChAFgroup.substring(ChAFgroup.lastIndexOf("/") + 1 ); 
if(ChAFgroup != null)
{
  if(Integer.valueOf(ChAFgroup) == 0)
  {
    g0asssel |= (chsel << Integer.valueOf(ChAFNum));
    g0chact |= (1 << Integer.valueOf(ChAFNum));
  }
  else if(Integer.valueOf(ChAFgroup) == 1)
  {
    g1asssel |= (chsel << Integer.valueOf(ChAFNum));
    g1chact |= (1 << Integer.valueOf(ChAFNum));
  }
  else if(Integer.valueOf(ChAFgroup) == 2)
  {
    g2asssel |= (chsel << Integer.valueOf(ChAFNum));
    g2chact |= (1 << Integer.valueOf(ChAFNum));
  }
  else if(Integer.valueOf(ChAFgroup) == 3)
  {
    g3asssel |= (chsel << Integer.valueOf(ChAFNum));
    g3chact |= (1 << Integer.valueOf(ChAFNum));
  }
}

    stringBuffer.append(TEXT_133);
    stringBuffer.append( ChAFgroup);
    stringBuffer.append(TEXT_134);
    stringBuffer.append( ChAFNum);
    stringBuffer.append(TEXT_135);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwselectchannelaf/0"));
    stringBuffer.append(TEXT_136);
    }}
    stringBuffer.append(TEXT_137);
     if (TotalChannels > 0) { 
    stringBuffer.append(TEXT_138);
    stringBuffer.append(TotalChannels);
    stringBuffer.append(TEXT_139);
     if ((GrpAChannels > 0) && (ChANum != null)) { 
    stringBuffer.append(TEXT_140);
    }
     if ((GrpAChannels > 1) && (ChBNum != null)) { 
    stringBuffer.append(TEXT_141);
    }
     if ((GrpAChannels > 2) && (ChCNum != null)) { 
    stringBuffer.append(TEXT_142);
    }
     if ((GrpAChannels > 3) && (ChDNum != null)) { 
    stringBuffer.append(TEXT_143);
    }
     if ((GrpAChannels > 4) && (ChENum != null)) { 
    stringBuffer.append(TEXT_144);
    }
     if((GrpAChannels > 5) && (ChFNum != null)) { 
    stringBuffer.append(TEXT_145);
    }
     if ((GrpAChannels > 6) && (ChGNum != null)){ 
    stringBuffer.append(TEXT_146);
    }
     if ((GrpAChannels > 7) && (ChHNum != null)) { 
    stringBuffer.append(TEXT_147);
    }
     if ((GrpBChannels > 0) && (ChINum != null)) { 
    stringBuffer.append(TEXT_148);
    }
     if ((GrpBChannels > 1)  && (ChJNum != null)){ 
    stringBuffer.append(TEXT_149);
    }
     if ((GrpBChannels > 2) && (ChKNum != null)) { 
    stringBuffer.append(TEXT_150);
    }
     if ((GrpBChannels > 3) && (ChLNum != null)) { 
    stringBuffer.append(TEXT_151);
    }
     if ((GrpBChannels > 4) && (ChMNum != null)) { 
    stringBuffer.append(TEXT_152);
    }
     if ((GrpBChannels > 5) && (ChNNum != null)) { 
    stringBuffer.append(TEXT_153);
    }
    stringBuffer.append(TEXT_154);
     if ((GrpBChannels > 6) && (ChONum != null))  { 
    stringBuffer.append(TEXT_155);
    }
     if ((GrpBChannels > 7) && (ChPNum != null)) { 
    stringBuffer.append(TEXT_156);
    }
     if ((GrpCChannels > 0) && (ChQNum != null)) { 
    stringBuffer.append(TEXT_157);
    }
     if ((GrpCChannels > 1)  && (ChRNum != null)){ 
    stringBuffer.append(TEXT_158);
    }
     if ((GrpCChannels > 2) && (ChSNum != null)) { 
    stringBuffer.append(TEXT_159);
    }
     if ((GrpCChannels > 3)  && (ChTNum != null)){ 
    stringBuffer.append(TEXT_160);
    }
     if ((GrpCChannels > 4) && (ChUNum != null)) { 
    stringBuffer.append(TEXT_161);
    }
     if ((GrpCChannels > 5) && (ChVNum != null)) { 
    stringBuffer.append(TEXT_162);
    }
     if ((GrpCChannels > 6)  && (ChWNum != null)){ 
    stringBuffer.append(TEXT_163);
    }
     if ((GrpCChannels > 7) && (ChXNum != null)) { 
    stringBuffer.append(TEXT_164);
    }
     if ((GrpDChannels > 0) && (ChYNum != null)) { 
    stringBuffer.append(TEXT_165);
    }
     if ((GrpDChannels > 1) && (ChZNum != null)) { 
    stringBuffer.append(TEXT_166);
    }
     if ((GrpDChannels > 2) && (ChAANum != null)){ 
    stringBuffer.append(TEXT_167);
    }
     if ((GrpDChannels > 3) && (ChABNum != null)) { 
    stringBuffer.append(TEXT_168);
    }
     if ((GrpDChannels > 4) && (ChACNum != null)) { 
    stringBuffer.append(TEXT_169);
    }
     if ((GrpDChannels > 5) && (ChADNum != null)) { 
    stringBuffer.append(TEXT_170);
    }
     if ((GrpDChannels > 6) && (ChAENum != null)) { 
    stringBuffer.append(TEXT_171);
    }
     if ((GrpDChannels > 7) && (ChAFNum != null)) { 
    stringBuffer.append(TEXT_172);
    }
    stringBuffer.append(TEXT_173);
    }
    stringBuffer.append(TEXT_174);
    stringBuffer.append(TotalChannels);
    stringBuffer.append(TEXT_175);
    stringBuffer.append( g0asssel);
    stringBuffer.append(TEXT_176);
    stringBuffer.append( g1asssel);
    stringBuffer.append(TEXT_177);
     if(Is1xDevice != 1 && Is42Device != 1) {
    stringBuffer.append(TEXT_178);
    stringBuffer.append( g2asssel);
    stringBuffer.append(TEXT_179);
    stringBuffer.append( g3asssel);
    stringBuffer.append(TEXT_180);
    }
    stringBuffer.append(TEXT_181);
    stringBuffer.append( GrpAChannels );
    stringBuffer.append(TEXT_182);
    stringBuffer.append( GrpBChannels );
    stringBuffer.append(TEXT_183);
     if(Is1xDevice != 1 && Is42Device != 1) {
    stringBuffer.append(TEXT_184);
    stringBuffer.append( GrpCChannels );
    stringBuffer.append(TEXT_185);
    stringBuffer.append( GrpDChannels );
    stringBuffer.append(TEXT_186);
    }
    stringBuffer.append(TEXT_187);
    stringBuffer.append( ((g3groupact >> 0 ) & 1));
    stringBuffer.append(TEXT_188);
    stringBuffer.append( ((g3groupact >> 1 ) & 1));
    stringBuffer.append(TEXT_189);
     if(Is1xDevice != 1  && Is42Device != 1) {
    stringBuffer.append(TEXT_190);
    stringBuffer.append( ((g3groupact >> 2 ) & 1));
    stringBuffer.append(TEXT_191);
    stringBuffer.append( ((g3groupact >> 3 ) & 1));
    stringBuffer.append(TEXT_192);
     }
    stringBuffer.append(TEXT_193);
    stringBuffer.append( ((g0chact >> 0 ) & 1));
    stringBuffer.append(TEXT_194);
    stringBuffer.append( ((g0chact >> 1 ) & 1));
    stringBuffer.append(TEXT_195);
    stringBuffer.append( ((g0chact >> 2 ) & 1));
    stringBuffer.append(TEXT_196);
    stringBuffer.append( ((g0chact >> 3 ) & 1));
    stringBuffer.append(TEXT_197);
    stringBuffer.append( ((g0chact >> 4 ) & 1));
    stringBuffer.append(TEXT_198);
    stringBuffer.append( ((g0chact >> 5 ) & 1));
    stringBuffer.append(TEXT_199);
    stringBuffer.append( ((g0chact >> 6 ) & 1));
    stringBuffer.append(TEXT_200);
    stringBuffer.append( ((g0chact >> 7 ) & 1));
    stringBuffer.append(TEXT_201);
    stringBuffer.append( ((g1chact >> 0 ) & 1));
    stringBuffer.append(TEXT_202);
    stringBuffer.append( ((g1chact >> 1 ) & 1));
    stringBuffer.append(TEXT_203);
    stringBuffer.append( ((g1chact >> 2 ) & 1));
    stringBuffer.append(TEXT_204);
    stringBuffer.append( ((g1chact >> 3 ) & 1));
    stringBuffer.append(TEXT_205);
    stringBuffer.append( ((g1chact >> 4 ) & 1));
    stringBuffer.append(TEXT_206);
    stringBuffer.append( ((g1chact >> 5 ) & 1));
    stringBuffer.append(TEXT_207);
    stringBuffer.append( ((g1chact >> 6 ) & 1));
    stringBuffer.append(TEXT_208);
    stringBuffer.append( ((g1chact >> 7 ) & 1));
    stringBuffer.append(TEXT_209);
     if(Is1xDevice != 1  && Is42Device != 1) {
    stringBuffer.append(TEXT_210);
    stringBuffer.append( ((g2chact >> 0 ) & 1));
    stringBuffer.append(TEXT_211);
    stringBuffer.append( ((g2chact >> 1 ) & 1));
    stringBuffer.append(TEXT_212);
    stringBuffer.append( ((g2chact >> 2 ) & 1));
    stringBuffer.append(TEXT_213);
    stringBuffer.append( ((g2chact >> 3 ) & 1));
    stringBuffer.append(TEXT_214);
    stringBuffer.append( ((g2chact >> 4 ) & 1));
    stringBuffer.append(TEXT_215);
    stringBuffer.append( ((g2chact >> 5 ) & 1));
    stringBuffer.append(TEXT_216);
    stringBuffer.append( ((g2chact >> 6 ) & 1));
    stringBuffer.append(TEXT_217);
    stringBuffer.append( ((g2chact >> 7 ) & 1));
    stringBuffer.append(TEXT_218);
    stringBuffer.append( ((g3chact >> 0 ) & 1));
    stringBuffer.append(TEXT_219);
    stringBuffer.append( ((g3chact >> 1 ) & 1));
    stringBuffer.append(TEXT_220);
    stringBuffer.append( ((g3chact >> 2 ) & 1));
    stringBuffer.append(TEXT_221);
    stringBuffer.append( ((g3chact >> 3 ) & 1));
    stringBuffer.append(TEXT_222);
    stringBuffer.append( ((g3chact >> 4 ) & 1));
    stringBuffer.append(TEXT_223);
    stringBuffer.append( ((g3chact >> 5 ) & 1));
    stringBuffer.append(TEXT_224);
    stringBuffer.append( ((g3chact >> 6 ) & 1));
    stringBuffer.append(TEXT_225);
    stringBuffer.append( ((g3chact >> 7 ) & 1));
    stringBuffer.append(TEXT_226);
    }
    stringBuffer.append(TEXT_227);
     if (app.getIntegerValue(AppBaseuri + "0/adc001_erwenablescansource/0") == 1){
    stringBuffer.append(TEXT_228);
    }  else {
    stringBuffer.append(TEXT_229);
    } 
    stringBuffer.append(TEXT_230);
     if (app.getIntegerValue(AppBaseuri +  "0/ADC001_erwtrgmode/0") == 1){
    stringBuffer.append(TEXT_231);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwtrgmode/1") == 1){
    stringBuffer.append(TEXT_232);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwtrgmode/2") == 1){
    stringBuffer.append(TEXT_233);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwtrgmode/3") == 1){
    stringBuffer.append(TEXT_234);
    } 
    stringBuffer.append(TEXT_235);
     if (app.getIntegerValue(AppBaseuri +  "0/ADC001_erwgatmode/0") == 1){
    stringBuffer.append(TEXT_236);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwgatmode/1") == 1){
    stringBuffer.append(TEXT_237);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwgatmode/2") == 1){
    stringBuffer.append(TEXT_238);
    } else if  (app.getIntegerValue(AppBaseuri + "0/ADC001_erwgatmode/3") == 1){
    stringBuffer.append(TEXT_239);
    } 
    stringBuffer.append(TEXT_240);
     if (app.getIntegerValue(AppBaseuri +  "0/adc001_erwtrgmode") > 0){
    stringBuffer.append(TEXT_241);
    } else {
    stringBuffer.append(TEXT_242);
    }
    stringBuffer.append(TEXT_243);
     if (app.getIntegerValue(AppBaseuri + "0/adc001_erwautoscanmode/0") == 1){
    stringBuffer.append(TEXT_244);
    } else {
    stringBuffer.append(TEXT_245);
    }
    stringBuffer.append(TEXT_246);
     if (app.getIntegerValue(AppBaseuri + "0/adc001_erwresultposition/0") == 1){
    stringBuffer.append(TEXT_247);
    } else {
    stringBuffer.append(TEXT_248);
    }
    stringBuffer.append(TEXT_249);
    stringBuffer.append( app.getIntegerValue(AppBaseuri + "0/adc001_erwresulteventenable/0"));
    stringBuffer.append(TEXT_250);
     if((app.getIntegerValue(AppBaseuri + "0/adc001_erwinputclasssel/0") == 1)){
    stringBuffer.append(TEXT_251);
    }
    else {
    stringBuffer.append(TEXT_252);
    }
    stringBuffer.append(TEXT_253);
    if (Is45Device == 1 || Is44Device == 1 )	{
    stringBuffer.append(TEXT_254);
     } 
    stringBuffer.append(TEXT_255);
    stringBuffer.append(TEXT_256);
    return stringBuffer.toString();
  }
}
