<?xml version="1.0" encoding="ASCII"?>
<ResourceModel:App xmi:version="2.0" xmlns:xmi="http://www.omg.org/XMI" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:ResourceModel="http://www.infineon.com/Davex/Resource.ecore" name="ADCGROUP001" description="This is ADC group app. It provides&#xA; ADC group specific,EMUX specific,&#xA;and group input class 0&#xA;and class 1 specific configurations.&#xA;This consumes ADC global app." descriptionURL="/doc/html/group___a_d_c_g_r_o_u_p001_app.html" mode="SHARABLE" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0">
  <upWardMappingList xsi:type="ResourceModel:ProxyResource" href="../../ADC001/1.0.22/adc001_0.app#//@consumed.44"/>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Select" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_chselect_vs" requiredSignalResource="adcgroupconfig/select">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Queue Request" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_queuereq_vs" requiredSignalResource="adcgroupconfig/gqueuereq">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Scan Request" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_scanreq_vs" requiredSignalResource="adcgroupconfig/gscanreq">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Sync Request" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_syncreq_vs" requiredSignalResource="adcgroupconfig/gsyncreq">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Global Signal" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_global_vs" requiredSignalResource="adcgroupconfig/global_signal">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Sample" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_gsample_vs" requiredSignalResource="adcgroupconfig/gsample">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="Equidistant Sampling Signal" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_garbcnt_vs" requiredSignalResource="adcgroupconfig/garbcnt">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="EMUX Control Line 0" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emux0_vs" requiredSignalResource="adcgroupconfig/emux0">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="EMUX Control Line 1" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emux1_vs" requiredSignalResource="adcgroupconfig/emux1">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:VirtualSignal" name="EMUX Control Line 2" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emux2_vs" requiredSignalResource="adcgroupconfig/emux2">
    <defaultValue xsi:type="ResourceModel:Blocked" value="Blocked"/>
    <localValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="Blocked"/>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Arbitset" evalFunction="&#xD;&#xA;        function ForwardMapping(){&#xD;&#xA;              var usemast = SCM.getResource(&quot;temp_arbit&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(usemast,CurrentValue);&#xD;&#xA;        }&#xD;&#xA;        function BackwardMapping(){&#xD;&#xA;              var usemast = SCM.getResource(&quot;temp_arbit&quot;);&#xD;&#xA;              var useValue = SCM.getIntValue(usemast);&#xD;&#xA;              SCM.setIntValue(currentResource,useValue);&#xD;&#xA;        }&#xD;&#xA;        " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwarbitset" downWardmappedList="//@consumed.2" maxValue="1" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to grey out the arbitration mode option in the UI.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="masterslave" evalFunction="&#xD;&#xA;        function ForwardMapping(){&#xD;&#xA;              var usemast = SCM.getResource(&quot;temp_mast&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(usemast,CurrentValue);&#xD;&#xA;        }&#xD;&#xA;        function BackwardMapping(){&#xD;&#xA;              var usemast = SCM.getResource(&quot;temp_mast&quot;);&#xD;&#xA;              var useValue = SCM.getIntValue(usemast);&#xD;&#xA;              SCM.setIntValue(currentResource,useValue);&#xD;&#xA;        }&#xD;&#xA;        " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwmasterorslave" downWardmappedList="//@consumed.22" maxValue="1" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This saves whether group is a sync slave.&#xD;
0: sync master&#xD;
1: sync slave</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Conversion Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(cmsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;              var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;              SCM.setIntValue(currentResource,cmsValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass0" downWardmappedList="//@consumed.40">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This allow the user to choose different conversion mode of VADC.
12-bit/10-bit/ 8-bit conversion or 10-bit fast compare mode.</toolTipHelpDescription>
    <item name="12 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;            if (cmsValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass0/0" downWardmappedList="//@consumed.40" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects 12 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;            if (cmsValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass0/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="8 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == -1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  } else if (cmsValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass0/2" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 8 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit FastCompare Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,5);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == 5) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass0/3" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit fast compare mode. It is applicable only for Fast Compare mode.</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Conversion Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(cmsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;              var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;              SCM.setIntValue(currentResource,cmsValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass1" downWardmappedList="//@consumed.45">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This allow the user to choose different conversion mode of VADC.
12-bit/10-bit/ 8-bit conversion or 10-bit fast compare mode.</toolTipHelpDescription>
    <item name="12 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass1/0" downWardmappedList="//@consumed.45" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects 12 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass1/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="8 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == -1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  } else if (cmsValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass1/2" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 8 bit standard conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit FastCompare Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmsRes,5);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;                  var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;                  if (cmsValue == 5) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwconversionmodeclass1/3" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit fast compare mode standard conversion</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Request app class 0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_requestappclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_requestappclass0&quot;);&#xD;&#xA;              var stcsValue = SCM.getIntValue(stcsRes);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwuseclass0" maxValue="1" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
    <toolTipHelpDescription>This allows to configure class 0</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Request app class 1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_requestappclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_requestappclass1&quot;);&#xD;&#xA;              var stcsValue = SCM.getIntValue(stcsRes);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwuseclass1" maxValue="1" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
    <toolTipHelpDescription>This allows to configure class 1</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Peripheral Bus Clock" evalFunction="&#xD;&#xA;        function ForwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;          var Val = SCM.getDoubleValue(Res);&#xD;&#xA;        }&#xD;&#xA;        function BackwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;          var Val = SCM.getDoubleValue(Res);&#xD;&#xA;          SCM.setDoubleValue(currentResource,Val);&#xD;&#xA;        }&#xD;&#xA;" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwperibusclock" maxValue="3e8" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="2"/>
    <localValue xsi:type="ResourceModel:StringValue" value="2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="78"/>
    <toolTipHelpDescription>The peripheral bus clock frequency</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Double Clock for MSB Conversion" evalFunction="&#xD;&#xA;        function ForwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;          var Val = SCM.getIntValue(Res);&#xD;&#xA;        }&#xD;&#xA;        function BackwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;          var Val = SCM.getIntValue(Res);&#xD;&#xA;          SCM.setIntValue(currentResource,Val);&#xD;&#xA;        }&#xD;&#xA;" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwdblclockmsb" maxValue="3e8" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="2"/>
    <localValue xsi:type="ResourceModel:StringValue" value="2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>If this is checked then MSB bit conversion will take two clock cycles.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Analog Clock" evalFunction="&#xD;&#xA;        function ForwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;          var Val = SCM.getDoubleValue(Res);&#xD;&#xA;        }&#xD;&#xA;        function BackwardMapping()&#xD;&#xA;        {&#xD;&#xA;          var Res = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;          var Val = SCM.getDoubleValue(Res);&#xD;&#xA;          SCM.setDoubleValue(currentResource,Val);&#xD;&#xA;        }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwselanalogclock" maxValue="3c" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="2"/>
    <localValue xsi:type="ResourceModel:StringValue" value="2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="1e"/>
    <toolTipHelpDescription>This is the actual Analog Clock Value that is possible.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Desired Sample time class0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getDoubleValue(currentResource);&#xD;&#xA;              SCM.setDoubleValue(DesiredSampleTimeRes,CurrentValue);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var resource = SCM.getResource(&quot;adcgroupconfig/giclass0/stcs&quot;);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;&#xD;&#xA;              var temp = (CurrentValue/tADCI) - 2;&#xD;&#xA;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=31;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC+15);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;              SCM.setIntValue(resource, STC);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;              SCM.setDoubleValue(currentResource,DesiredSampleTimeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwdesiredsampletimeclass0" downWardmappedList="//@consumed.6 //@consumed.39" maxValue="2198" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <localValue xsi:type="ResourceModel:StringValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <toolTipHelpDescription>This is the desired sample time
used for sampling in standard conversion.
Sample time= (2+STC)*Tadci, where STC - Additional Clock Cycle.
Total conversion time depends on sample time and result width.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample time class0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_sampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(stcsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;&#xD;&#xA;              var actanalogclkRes = SCM.getResource(&quot;adcgroup001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkValue = SCM.getDoubleValue(actanalogclkRes);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkValue;&#xD;&#xA;              var temp = (DesiredSampleTimeValue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC*16);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;&#xD;&#xA;              var ts = (2+STC)*tADCI;&#xD;&#xA;&#xD;&#xA;              SCM.setDoubleValue(currentResource,ts);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwsampletimeclass0" downWardmappedList="//@consumed.7" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="4050aaaaaaaaaaab"/>
    <toolTipHelpDescription>This is the Actual sample time
 used for sampling in standard conversion</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Conversion Time" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue1 = SCM.getDoubleValue(actanalogclk1);&#xD;&#xA;              var preclock1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval1 = SCM.getDoubleValue(preclock1);&#xD;&#xA;              var Res41 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val41 = SCM.getIntValue(Res41);&#xD;&#xA;              var Res51 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val51 = SCM.getIntValue(Res51);&#xD;&#xA;              var convmode1 = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;              var convmodeval1 = SCM.getIntValue(convmode1);&#xD;&#xA;              var stRes1 = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var stvalue1 = SCM.getIntValue(stRes1);&#xD;&#xA;            }&#xD;&#xA;&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass0&quot;);&#xD;&#xA;              var stvalue = SCM.getDoubleValue(stRes);&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;              var preclock = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval = SCM.getDoubleValue(preclock);&#xD;&#xA;              var Res4 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val4 = SCM.getIntValue(Res4);&#xD;&#xA;              var Res5 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val5 = SCM.getIntValue(Res5);&#xD;&#xA;              var convmode = SCM.getResource(&quot;adcgroupconfig/giclass0/cms&quot;);&#xD;&#xA;              var convmodeval = SCM.getIntValue(convmode);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;              var tADC = 2000/preclockval;&#xD;&#xA;              var temp = (stvalue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;                } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;&#xD;&#xA;              STC= Math.floor( (temp) / (16) );&#xD;&#xA;              STC=(STC*16);&#xD;&#xA;                } else {&#xD;&#xA;                  STC= Math.floor(temp);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;              var temp_N = 0;&#xD;&#xA;              var temp_DM = 0;&#xD;&#xA;              var temp_PC = 0;&#xD;&#xA;              var ActSampling = 0;&#xD;&#xA;&#xD;&#xA;              if(val4 == 1) {&#xD;&#xA;                temp_DM =1;&#xD;&#xA;                }&#xD;&#xA;              if(val5 == 0) {&#xD;&#xA;                temp_PC =2;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;              if(convmodeval == 0) {&#xD;&#xA;                temp_N =12;&#xD;&#xA;                } else if (convmodeval == 1) {&#xD;&#xA;                temp_N =10;&#xD;&#xA;                } else if (convmodeval == 2) {&#xD;&#xA;                temp_N =8;&#xD;&#xA;                }&#xD;&#xA;&#x9;/******** ADC conversion time calculation for XMC 1X Device - START  ******/&#xD;&#xA;&#x9;&#x9;var DeviceCheck = SCM.getSoftwareId().substring(0,1).compareTo(&quot;1&quot;);&#xD;&#xA;        var tsh = 1000/32;   //32 MHz S&amp;H Clock&#xD;&#xA;        &#xD;&#xA;        if(convmodeval == 5) &#xD;&#xA;        {&#xD;&#xA;                ActSampling = (2+STC+2)*tADCI + tADC;&#xD;&#xA;        }&#x9;&#xD;&#xA;        else&#xD;&#xA;        {&#xD;&#xA;             if(DeviceCheck != 0)&#xD;&#xA;             {&#x9;&#x9;         &#xD;&#xA;                ActSampling = (2+STC+temp_N+temp_DM+temp_PC)*tADCI + tADC;&#xD;&#xA;             }&#xD;&#xA;             else&#xD;&#xA;             {&#xD;&#xA;                ActSampling = (2+STC)*tADCI + 4*tsh + (temp_N + 8)*tsh + 5*(tADC/2);&#xD;&#xA;             }&#xD;&#xA;         }&#xD;&#xA;&#xD;&#xA;     /******** ADC conversion time calculation for XMC 1X Device - END  ******/&#xD;&#xA;&#xD;&#xA;              SCM.setDoubleValue(currentResource,ActSampling);&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwconversiontimeclass0" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="226"/>
    <toolTipHelpDescription>This is actual conversion time for standard conversion configured in class 0</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample Time Control Class 0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/stcs&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/stcs&quot;);&#xD;&#xA;              var stcsValue = SCM.getIntValue(stcsRes);&#xD;&#xA;              SCM.setIntValue(currentResource,stcsValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwactualsampletimecontrolclass0" maxValue="20" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Additional clock cycles for
sample time control for
 standard conversions configured in class 0</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Desired Sample time class1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getDoubleValue(currentResource);&#xD;&#xA;              SCM.setDoubleValue(DesiredSampleTimeRes,CurrentValue);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var resource = SCM.getResource(&quot;adcgroupconfig/giclass1/stcs&quot;);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;&#xD;&#xA;              var temp = (CurrentValue/tADCI) - 2;&#xD;&#xA;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=31;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC+15);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;              SCM.setIntValue(resource, STC);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;              SCM.setDoubleValue(currentResource,DesiredSampleTimeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwdesiredsampletimeclass1" downWardmappedList="//@consumed.9 //@consumed.44" maxValue="2198" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <localValue xsi:type="ResourceModel:StringValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <toolTipHelpDescription>This is the desired sample time
used for sampling in standard conversion.
Sample time= (2+STC)*Tadci, where STC - Additional Clock Cycle.
Total conversion time depends on sample time and result width.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample time class1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_sampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(stcsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;&#xD;&#xA;              var actanalogclkRes = SCM.getResource(&quot;adcgroup001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkValue = SCM.getDoubleValue(actanalogclkRes);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkValue;&#xD;&#xA;              var temp = (DesiredSampleTimeValue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC*16);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;&#xD;&#xA;              var ts = (2+STC)*tADCI;&#xD;&#xA;&#xD;&#xA;              SCM.setDoubleValue(currentResource,ts);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwsampletimeclass1" downWardmappedList="//@consumed.11" maxValue="FFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="4050aaaaaaaaaaab"/>
    <toolTipHelpDescription>This is the Actual sample time
 used for sampling in standard conversion</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Conversion Time" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue1 = SCM.getDoubleValue(actanalogclk1);&#xD;&#xA;              var preclock1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval1 = SCM.getDoubleValue(preclock1);&#xD;&#xA;              var Res41 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val41 = SCM.getIntValue(Res41);&#xD;&#xA;              var Res51 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val51 = SCM.getIntValue(Res51);&#xD;&#xA;              var convmode1 = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;              var convmodeval1 = SCM.getIntValue(convmode1);&#xD;&#xA;              var stRes1 = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var stvalue1 = SCM.getIntValue(stRes1);&#xD;&#xA;            }&#xD;&#xA;&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stRes = SCM.getResource(&quot;adcgroup001_desiredsampletimeclass1&quot;);&#xD;&#xA;              var stvalue = SCM.getDoubleValue(stRes);&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;              var preclock = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval = SCM.getDoubleValue(preclock);&#xD;&#xA;              var Res4 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val4 = SCM.getIntValue(Res4);&#xD;&#xA;              var Res5 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val5 = SCM.getIntValue(Res5);&#xD;&#xA;              var convmode = SCM.getResource(&quot;adcgroupconfig/giclass1/cms&quot;);&#xD;&#xA;              var convmodeval = SCM.getIntValue(convmode);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;              var tADC = 2000/preclockval;&#xD;&#xA;              var temp = (stvalue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;                } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;&#xD;&#xA;              STC= Math.floor( (temp) / (16) );&#xD;&#xA;              STC=(STC*16);&#xD;&#xA;                } else {&#xD;&#xA;                  STC= Math.floor(temp);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;              var temp_N = 0;&#xD;&#xA;              var temp_DM = 0;&#xD;&#xA;              var temp_PC = 0;&#xD;&#xA;              var ActSampling = 0;&#xD;&#xA;&#xD;&#xA;              if(val4 == 1) {&#xD;&#xA;                temp_DM =1;&#xD;&#xA;                }&#xD;&#xA;              if(val5 == 0) {&#xD;&#xA;                temp_PC =2;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;              if(convmodeval == 0) {&#xD;&#xA;                temp_N =12;&#xD;&#xA;                } else if (convmodeval == 1) {&#xD;&#xA;                temp_N =10;&#xD;&#xA;                } else if (convmodeval == 2) {&#xD;&#xA;                temp_N =8;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;      &#x9;/******** ADC conversion time calculation for XMC 1X Device - START  ******/&#xD;&#xA;&#x9;&#x9;var DeviceCheck = SCM.getSoftwareId().substring(0,1).compareTo(&quot;1&quot;);&#xD;&#xA;        var tsh = 1000/32;   //32 MHz S&amp;H Clock&#xD;&#xA;              if(convmodeval == 5) {&#xD;&#xA;                ActSampling = (2+STC+2)*tADCI + tADC;&#xD;&#xA;&#xD;&#xA;               } else {&#xD;&#xA;              if(DeviceCheck != 0)&#xD;&#xA;              {&#xD;&#xA;                   ActSampling = (2+STC+temp_N+temp_DM+temp_PC)*tADCI + tADC;&#xD;&#xA;              }&#xD;&#xA;              else&#xD;&#xA;              {&#xD;&#xA;                ActSampling = (2+STC)*tADCI + 4*tsh + (temp_N + 8)*tsh + 5*(tADC/2);&#xD;&#xA;              }&#xD;&#xA;            }&#xD;&#xA;              SCM.setDoubleValue(currentResource,ActSampling);&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwconversiontimeclass1" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="226"/>
    <toolTipHelpDescription>This is actual conversion time for standard conversion configured in class 1</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample Time Control Class 1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/stcs&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroupconfig/giclass1/stcs&quot;);&#xD;&#xA;              var stcsValue = SCM.getIntValue(stcsRes);&#xD;&#xA;              SCM.setIntValue(currentResource,stcsValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwactualsampletimecontrolclass1" maxValue="20" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Additional clock cycles for
sample time control for
 standard conversions configured in class 1</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Conversion Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(cmsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmsRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;              var cmsValue = SCM.getIntValue(cmsRes);&#xD;&#xA;              SCM.setIntValue(currentResource,cmsValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass0" downWardmappedList="//@consumed.42">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This allow the user to choose different external multiplexer conversion mode of VADC.
12-bit/10-bit/ 8-bit conversion or 10-bit fast compare mode.</toolTipHelpDescription>
    <item name="12 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass0/0" downWardmappedList="//@consumed.42" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects 12 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass0/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="8 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == -1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  } else if (cmeValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass0/2" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 8 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit FastCompare Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,5);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;            if (cmeValue == 5) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass0/3" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit fast compare mode EMUX conversion</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Conversion Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(cmeRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;              var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;              SCM.setIntValue(currentResource,cmeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass1" downWardmappedList="//@consumed.47">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This allow the user to choose different external multiplexer conversion mode of VADC.
12-bit/10-bit/ 8-bit conversion or 10-bit fast compare mode.</toolTipHelpDescription>
    <item name="12 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass1/0" downWardmappedList="//@consumed.47" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects 12 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass1/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="8 Bit Conversion" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == -1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  } else if (cmeValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass1/2" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 8 bit EMUX conversion mode</toolTipHelpDescription>
    </item>
    <item name="10 Bit FastCompare Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(cmeRes,5);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var cmeRes = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;                  var cmeValue = SCM.getIntValue(cmeRes);&#xD;&#xA;                  if (cmeValue == 5) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxconversionmodeclass1/3" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects 10 bit fast compare mode EMUX conversion</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="EMUX Desired Sample time class0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getDoubleValue(currentResource);&#xD;&#xA;              SCM.setDoubleValue(DesiredSampleTimeRes,CurrentValue);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var resource = SCM.getResource(&quot;adcgroupconfig/giclass0/stce&quot;);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;&#xD;&#xA;              var temp = (CurrentValue/tADCI) - 2;&#xD;&#xA;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=31;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC+15);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;              SCM.setIntValue(resource, STC);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;              SCM.setDoubleValue(currentResource,DesiredSampleTimeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxdesiredsampletimeclass0" downWardmappedList="//@consumed.14 //@consumed.41" maxValue="2198" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <localValue xsi:type="ResourceModel:StringValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <toolTipHelpDescription>This is the desired sample time
used for sampling in EMUX.
Sample time= (2+STC)*Tadci, where STC - Additional clock cycle.
Total conversion time depends on sample time and result width.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample time emux class0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_emuxsampletimeclass0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(stcsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;&#xD;&#xA;              var actanalogclkRes = SCM.getResource(&quot;adcgroup001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkValue = SCM.getDoubleValue(actanalogclkRes);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkValue;&#xD;&#xA;              var temp = (DesiredSampleTimeValue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC*16);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;&#xD;&#xA;              var ts = (2+STC)*tADCI;&#xD;&#xA;&#xD;&#xA;              SCM.setDoubleValue(currentResource,ts);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxsampletimeclass0" downWardmappedList="//@consumed.15" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="4050aaaaaaaaaaab"/>
    <toolTipHelpDescription>This is the Actual sample time
 used for sampling in EMUX conversion</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Conversion Time" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue1 = SCM.getDoubleValue(actanalogclk1);&#xD;&#xA;              var preclock1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval1 = SCM.getDoubleValue(preclock1);&#xD;&#xA;              var Res41 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val41 = SCM.getIntValue(Res41);&#xD;&#xA;              var Res51 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val51 = SCM.getIntValue(Res51);&#xD;&#xA;              var convmode1 = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;              var convmodeval1 = SCM.getIntValue(convmode1);&#xD;&#xA;              var stRes1 = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var stvalue1 = SCM.getIntValue(stRes1);&#xD;&#xA;            }&#xD;&#xA;&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass0&quot;);&#xD;&#xA;              var stvalue = SCM.getDoubleValue(stRes);&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;              var preclock = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval = SCM.getDoubleValue(preclock);&#xD;&#xA;              var Res4 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val4 = SCM.getIntValue(Res4);&#xD;&#xA;              var Res5 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val5 = SCM.getIntValue(Res5);&#xD;&#xA;              var convmode = SCM.getResource(&quot;adcgroupconfig/giclass0/cme&quot;);&#xD;&#xA;              var convmodeval = SCM.getIntValue(convmode);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;              var tADC = 2000/preclockval;&#xD;&#xA;              var temp = (stvalue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;                } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;&#xD;&#xA;              STC= Math.floor( (temp) / (16) );&#xD;&#xA;              STC=(STC*16);&#xD;&#xA;                } else {&#xD;&#xA;                  STC= Math.floor(temp);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;              var temp_N = 0;&#xD;&#xA;              var temp_DM = 0;&#xD;&#xA;              var temp_PC = 0;&#xD;&#xA;              var ActSampling = 0;&#xD;&#xA;&#xD;&#xA;              if(val4 == 1) {&#xD;&#xA;                temp_DM =1;&#xD;&#xA;                }&#xD;&#xA;              if(val5 == 0) {&#xD;&#xA;                temp_PC =2;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;              if(convmodeval == 0) {&#xD;&#xA;                temp_N =12;&#xD;&#xA;                } else if (convmodeval == 1) {&#xD;&#xA;                temp_N =10;&#xD;&#xA;                } else if (convmodeval == 2) {&#xD;&#xA;                temp_N =8;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;       &#x9;/******** ADC conversion time calculation for XMC 1X Device - START  ******/&#xD;&#xA;&#x9;&#x9;var DeviceCheck = SCM.getSoftwareId().substring(0,1).compareTo(&quot;1&quot;);&#xD;&#xA;        var tsh = 1000/32;   //32 MHz S&amp;H Clock&#xD;&#xA;              if(convmodeval == 5) {&#xD;&#xA;                ActSampling = (2+STC+2)*tADCI + tADC;&#xD;&#xA;&#xD;&#xA;               } else {&#xD;&#xA;              if(DeviceCheck != 0)&#xD;&#xA;              {&#x9;&#x9;         &#xD;&#xA;                 ActSampling = (2+STC+temp_N+temp_DM+temp_PC)*tADCI + tADC;&#xD;&#xA;               }&#xD;&#xA;              else&#xD;&#xA;              {&#xD;&#xA;                ActSampling = (2+STC)*tADCI + 4*tsh + (temp_N + 8)*tsh + 5*(tADC/2);&#xD;&#xA;              }&#xD;&#xA;             }&#xD;&#xA; &#x9;/******** ADC conversion time calculation for XMC 1X Device - END  ******/&#xD;&#xA;              SCM.setDoubleValue(currentResource,ActSampling);&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxconversiontimeclass0" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="226"/>
    <toolTipHelpDescription>This is actual conversion time for EMUX configured in class 0</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual EMUX Sample Time" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stceRes = SCM.getResource(&quot;adcgroupconfig/giclass0/stce&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stceRes = SCM.getResource(&quot;adcgroupconfig/giclass0/stce&quot;);&#xD;&#xA;              var stceValue = SCM.getIntValue(stceRes);&#xD;&#xA;              SCM.setIntValue(currentResource,stceValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxactualsampletimecontrolclass0" maxValue="20" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Additional clock cycles for
sample time control for
EMUX conversions configured in class 0</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="EMUX Desired Sample time class1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getDoubleValue(currentResource);&#xD;&#xA;              SCM.setDoubleValue(DesiredSampleTimeRes,CurrentValue);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var resource = SCM.getResource(&quot;adcgroupconfig/giclass1/stce&quot;);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;&#xD;&#xA;              var temp = (CurrentValue/tADCI) - 2;&#xD;&#xA;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=31;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC+15);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;              SCM.setIntValue(resource, STC);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;              SCM.setDoubleValue(currentResource,DesiredSampleTimeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxdesiredsampletimeclass1" downWardmappedList="//@consumed.18 //@consumed.46" maxValue="2198" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <localValue xsi:type="ResourceModel:StringValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <toolTipHelpDescription>This is the desired sample time
used for sampling in EMUX.
Sample time= (2+STC)*Tadci, where STC - Additional clock cycle.
Total conversion time depends on sample time and result width.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual Sample time emux class1" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stcsRes = SCM.getResource(&quot;adcgroup001_emuxsampletimeclass1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(stcsRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var DesiredSampleTimeRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var DesiredSampleTimeValue = SCM.getDoubleValue(DesiredSampleTimeRes);&#xD;&#xA;&#xD;&#xA;              var actanalogclkRes = SCM.getResource(&quot;adcgroup001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkValue = SCM.getDoubleValue(actanalogclkRes);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkValue;&#xD;&#xA;              var temp = (DesiredSampleTimeValue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;              } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;                STC= Math.floor( (temp) / (16) );&#xD;&#xA;                STC=(STC*16);&#xD;&#xA;              } else {&#xD;&#xA;                STC= Math.floor(temp);&#xD;&#xA;              }&#xD;&#xA;&#xD;&#xA;              var ts = (2+STC)*tADCI;&#xD;&#xA;&#xD;&#xA;              SCM.setDoubleValue(currentResource,ts);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxsampletimeclass1" downWardmappedList="//@consumed.19" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="4050aaaaaaaaaaab"/>
    <toolTipHelpDescription>This is the Actual number of additional clock cycles
 used for sampling in emux conversion</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Conversion Time" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue1 = SCM.getDoubleValue(actanalogclk1);&#xD;&#xA;              var preclock1 = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval1 = SCM.getDoubleValue(preclock1);&#xD;&#xA;              var Res41 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val41 = SCM.getIntValue(Res41);&#xD;&#xA;              var Res51 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val51 = SCM.getIntValue(Res51);&#xD;&#xA;              var convmode1 = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;              var convmodeval1 = SCM.getIntValue(convmode1);&#xD;&#xA;              var stRes1 = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var stvalue1 = SCM.getIntValue(stRes1);&#xD;&#xA;            }&#xD;&#xA;&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var stRes = SCM.getResource(&quot;adcgroup001_emuxdesiredsampletimeclass1&quot;);&#xD;&#xA;              var stvalue = SCM.getDoubleValue(stRes);&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;              var preclock = SCM.getResource(&quot;adcglobalapp/adcglob001_irwperibusclock&quot;);&#xD;&#xA;              var preclockval = SCM.getDoubleValue(preclock);&#xD;&#xA;              var Res4 = SCM.getResource(&quot;adcglobalapp/adcglob001_erwdblclockmsb&quot;);&#xD;&#xA;              var val4 = SCM.getIntValue(Res4);&#xD;&#xA;              var Res5 = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var val5 = SCM.getIntValue(Res5);&#xD;&#xA;              var convmode = SCM.getResource(&quot;adcgroupconfig/giclass1/cme&quot;);&#xD;&#xA;              var convmodeval = SCM.getIntValue(convmode);&#xD;&#xA;&#xD;&#xA;              var tADCI = 1000/actanalogclkvalue;&#xD;&#xA;              var tADC = 2000/preclockval;&#xD;&#xA;              var temp = (stvalue/tADCI) - 2;&#xD;&#xA;              var STC = 0;&#xD;&#xA;&#xD;&#xA;              if(temp &lt; 0) {&#xD;&#xA;                STC =0;&#xD;&#xA;              } else if (temp > 256) {&#xD;&#xA;                STC=256;&#xD;&#xA;                } else if ((temp > 16) &amp;&amp; (temp &lt;= 256) ) {&#xD;&#xA;&#xD;&#xA;              STC= Math.floor( (temp) / (16) );&#xD;&#xA;              STC=(STC*16);&#xD;&#xA;                } else {&#xD;&#xA;                  STC= Math.floor(temp);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;              var temp_N = 0;&#xD;&#xA;              var temp_DM = 0;&#xD;&#xA;              var temp_PC = 0;&#xD;&#xA;              var ActSampling = 0;&#xD;&#xA;&#xD;&#xA;              if(val4 == 1) {&#xD;&#xA;                temp_DM =1;&#xD;&#xA;                }&#xD;&#xA;              if(val5 == 0) {&#xD;&#xA;                temp_PC =2;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;              if(convmodeval == 0) {&#xD;&#xA;                temp_N =12;&#xD;&#xA;                } else if (convmodeval == 1) {&#xD;&#xA;                temp_N =10;&#xD;&#xA;                } else if (convmodeval == 2) {&#xD;&#xA;                temp_N =8;&#xD;&#xA;                }&#xD;&#xA;&#xD;&#xA;    &#x9;/******** ADC conversion time calculation for XMC 1X Device - START  ******/&#xD;&#xA;&#x9;&#x9;var DeviceCheck = SCM.getSoftwareId().substring(0,1).compareTo(&quot;1&quot;);&#xD;&#xA;        var tsh = 1000/32;   //32 MHz S&amp;H Clock&#xD;&#xA;              if(convmodeval == 5) {&#xD;&#xA;                ActSampling = (2+STC+2)*tADCI + tADC;&#xD;&#xA;&#xD;&#xA;               } else {&#xD;&#xA;               if(DeviceCheck != 0)&#xD;&#xA;               {&#xD;&#xA;                  ActSampling = (2+STC+temp_N+temp_DM+temp_PC)*tADCI + tADC;&#xD;&#xA;               }&#xD;&#xA;               else&#xD;&#xA;               {&#xD;&#xA;                 ActSampling = (2+STC)*tADCI + 4*tsh + (temp_N + 8)*tsh + 5*(tADC/2);&#xD;&#xA;               }&#xD;&#xA;              }&#xD;&#xA;        /******** ADC conversion time calculation for XMC 1X Device - END  ******/&#xD;&#xA;              SCM.setDoubleValue(currentResource,ActSampling);&#xD;&#xA;          }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxconversiontimeclass1" maxValue="FFFFFFFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="226"/>
    <toolTipHelpDescription>This is actual conversion time for EMUX configured in class 1</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Actual EMUX Sample Time" evalFunction="&#xD;&#xA;      function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stceRes = SCM.getResource(&quot;adcgroupconfig/giclass1/stce&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var actanalogclk = SCM.getResource(&quot;adcglobalapp/adcglob001_irwselanalogclock&quot;);&#xD;&#xA;              var actanalogclkvalue = SCM.getDoubleValue(actanalogclk);&#xD;&#xA;&#xD;&#xA;              var stceRes = SCM.getResource(&quot;adcgroupconfig/giclass1/stce&quot;);&#xD;&#xA;              var stceValue = SCM.getIntValue(stceRes);&#xD;&#xA;              SCM.setIntValue(currentResource,stceValue);&#xD;&#xA;            }&#xD;&#xA;    " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxactualsampletimecontrolclass1" maxValue="20" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Additional clock cycles for
sample time control for
 EMUX conversions configured in class 1</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Post Calibration" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var dpcalRes = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(dpcalRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var dpcalRes = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;              var dpcalValue = SCM.getIntValue(dpcalRes);&#xD;&#xA;              SCM.setIntValue(currentResource,dpcalValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwdatapostcalibration" downWardmappedList="//@consumed.34" multipleSelections="true">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to enable or disable post calibration.
This will compensate the effects of drifting parameters.
For 12 bit conversion, post calibration is executed
after converting channel. </toolTipHelpDescription>
    <item name="Enable" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var dpcalRes = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(dpcalRes,0);&#xD;&#xA;                  }&#xD;&#xA;                  else&#xD;&#xA;                  {&#xD;&#xA;                    SCM.setIntValue(dpcalRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var dpcalRes = SCM.getResource(&quot;adcgroupconfig/globcfg_dpcal&quot;);&#xD;&#xA;                  var dpcalValue = SCM.getIntValue(dpcalRes);&#xD;&#xA;                  if (dpcalValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwdatapostcalibration/0" downWardmappedList="//@consumed.34" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects automatic post calibration or
No post calibration after
each econversion.</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Arbitration Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(arbmRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;              var arbmValue = SCM.getIntValue(arbmRes);&#xD;&#xA;              SCM.setIntValue(currentResource,arbmValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwdataarbitrationmode" downWardmappedList="//@consumed.36">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to select arbitration mode</toolTipHelpDescription>
    <item name="Runs Permanently" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arbmRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;                  var arbmValue = SCM.getIntValue(arbmRes);&#xD;&#xA;            if (arbmValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwdataarbitrationmode/0" downWardmappedList="//@consumed.36" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects Arbiter runs permanently mode.
 In this mode, synchronized conversions of more than one ADC kernel are possible.</toolTipHelpDescription>
    </item>
    <item name="Starts With Conversion Request" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arbmRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arbmRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbm&quot;);&#xD;&#xA;                  var arbmValue = SCM.getIntValue(arbmRes);&#xD;&#xA;                  if (arbmValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwdataarbitrationmode/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects Arbiter runs if atleast one conversion request is pending mode.
 In this mode the arbiter stops after an arbitration round
 when no conversion request have been found pending any more.</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Group Specific Boundary0" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var boundary0Res = SCM.getResource(&quot;adcgroupconfig/gbound/boundary0&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(boundary0Res,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var boundary0Res = SCM.getResource(&quot;adcgroupconfig/gbound/boundary0&quot;);&#xD;&#xA;              var boundary0Value = SCM.getIntValue(boundary0Res);&#xD;&#xA;              SCM.setIntValue(currentResource,boundary0Value);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwgrpboundary0" downWardmappedList="//@consumed.49" maxValue="FFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Standard Mode: This is boundary0 value which
is compared against the left-aligned
conversion result. For 10/8-bit results,
the lower 2/4 bits must be zero.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="Group Specific Boundary1" evalFunction="&#xD;&#xA;      function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var boundary1Res = SCM.getResource(&quot;adcgroupconfig/gbound/boundary1&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(boundary1Res,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var boundary1Res = SCM.getResource(&quot;adcgroupconfig/gbound/boundary1&quot;);&#xD;&#xA;              var boundary1Value = SCM.getIntValue(boundary1Res);&#xD;&#xA;              SCM.setIntValue(currentResource,boundary1Value);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwgrpboundary1" downWardmappedList="//@consumed.50" maxValue="FFF" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>Standard Mode: This is boundary1 value which
is compared against the left-aligned
conversion result. For 10/8-bit results,
the lower 2/4 bits must be zero.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="Arbitration Round Length(Slots Per Round)" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(arblenRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;              var arblenValue = SCM.getIntValue(arblenRes);&#xD;&#xA;              SCM.setIntValue(currentResource,arblenValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwarbitrationround" downWardmappedList="//@consumed.37">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to select number of arbitration slots per arbitration round</toolTipHelpDescription>
    <item name="4 Slots" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arblenRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var arblenValue = SCM.getIntValue(arblenRes);&#xD;&#xA;                  if (arblenValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwarbitrationround/0" downWardmappedList="//@consumed.37" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>4 arbitration slots per round</toolTipHelpDescription>
    </item>
    <item name="8 Slots" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arblenRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var arblenValue = SCM.getIntValue(arblenRes);&#xD;&#xA;                  if (arblenValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwarbitrationround/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>8 arbitration slots per round</toolTipHelpDescription>
    </item>
    <item name="16 Slots" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arblenRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var arblenValue = SCM.getIntValue(arblenRes);&#xD;&#xA;                  if (arblenValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwarbitrationround/2" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>16 arbitration slots per round</toolTipHelpDescription>
    </item>
    <item name="20 Slots" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(arblenRes,3);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var arblenRes = SCM.getResource(&quot;adcgroupconfig/garbcfg/arbrnd&quot;);&#xD;&#xA;                  var arblenValue = SCM.getIntValue(arblenRes);&#xD;&#xA;                  if (arblenValue == 3) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                 } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwarbitrationround/3" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>20 arbitration slots per round</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="External MUX Coding Scheme" evalFunction="&#xD;&#xA;      function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(emxcodRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;              var emxcodValue = SCM.getIntValue(emxcodRes);&#xD;&#xA;              SCM.setIntValue(currentResource,emxcodValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcodingscheme" downWardmappedList="//@consumed.52">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to select external MUX coding scheme</toolTipHelpDescription>
    <item name="Binary Code" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emxcodRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;                  var emxcodValue = SCM.getIntValue(emxcodRes);&#xD;&#xA;                  if (emxcodValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcodingscheme/0" downWardmappedList="//@consumed.52" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>Selects Binary Code as EMUX coding scheme.
Binary Code: 000|001|010|011|100|101|110|111</toolTipHelpDescription>
    </item>
    <item name="Gray Code" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emxcodRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emxcodRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emxcod&quot;);&#xD;&#xA;                  var emxcodValue = SCM.getIntValue(emxcodRes);&#xD;&#xA;            if (emxcodValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcodingscheme/1" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:StringValue" value="0"/>
      <toolTipHelpDescription>Selects Gray Code as EMUX coding scheme
Gray Code: 000|001|011|010|110|111|101|100</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="External MUX Mode" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              var ioappRes0 = SCM.getResource(&quot;emuxctrl0/IO002_erwOutputMode/0&quot;);&#xD;&#xA;              var ioappRes1 = SCM.getResource(&quot;emuxctrl1/IO002_erwOutputMode/0&quot;);&#xD;&#xA;              var ioappRes2 = SCM.getResource(&quot;emuxctrl2/IO002_erwOutputMode/0&quot;);&#xD;&#xA;&#xD;&#xA;              var dep = &quot;DependentApp&quot;;&#xD;&#xA;              var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;              SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;&#xD;&#xA;              SCM.setIntValue(emuxmodeRes,CurrentValue);&#xD;&#xA;              SCM.setIntValue(ioappRes0,1);&#xD;&#xA;              SCM.setIntValue(ioappRes1,1);&#xD;&#xA;              SCM.setIntValue(ioappRes2,1);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;              var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;              SCM.setIntValue(currentResource,emuxmodeValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxmode" downWardmappedList="//@consumed.23 //@consumed.53 //@consumed.60 //@consumed.62 //@consumed.64">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This is to select external MUX mode.
When the selected channel is encountered,
Sequence mode automatically converts all
 configured external channels,
Single-step mode converts one external
channel of the configured sequence,
Steady mode converts the configured external channel.
Software mode switches off the HW control
of an external multiplexer.</toolTipHelpDescription>
    <item name="Software Control" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxmodeRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;            if (emuxmodeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxmode/0" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25 //@consumed.53" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>This mode disables the automatic EMUX control.
 This mode is used when there is no external multiplexer.</toolTipHelpDescription>
    </item>
    <item name="Steady Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxmodeRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;            if (emuxmodeValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxmode/1" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>This mode converts the configured external channel
when the selected channel is encountered</toolTipHelpDescription>
    </item>
    <item name="Single Step Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxmodeRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;            if (emuxmodeValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxmode/2" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>This mode converts one external channel of the configured sequence
when the selected channel is encountered</toolTipHelpDescription>
    </item>
    <item name="Sequence Mode" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxmodeRes,3);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;            if (emuxmodeValue == 3) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxmode/3" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>This mode automatically converts all configured external channels
when the selected channel is encountered</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:EnumerationParameter" name="EMUX Control Pins" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;              var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;              var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;              var dep = &quot;DependentApp&quot;;&#xD;&#xA;              var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;              SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;              var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;              SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;              var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;              SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;              SCM.setIntValue(emuxRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;              var emuxValue = SCM.getIntValue(emuxRes);&#xD;&#xA;&#xD;&#xA;              var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;              var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;              SCM.setIntValue(currentResource,emuxValue);&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcontrolpin" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25 //@consumed.21">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="3"/>
    <toolTipHelpDescription>This allows to select number of pins for EMUX control line</toolTipHelpDescription>
    <item name="0" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;&#xD;&#xA;                  if(CurrentValue == 0){&#xD;&#xA;                    SCM.setIntValue(emuxRes,0);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var emuxValue = SCM.getIntValue(emuxRes);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;                  if (emuxmodeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }&#xD;&#xA;                  else if (emuxValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcontrolpin/zero" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25 //@consumed.21" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <toolTipHelpDescription>EMUX is disabled.</toolTipHelpDescription>
    </item>
    <item name="1" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxRes,1);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var emuxValue = SCM.getIntValue(emuxRes);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;                  if (emuxmodeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                  else if (emuxValue == 1) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcontrolpin/0" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>Selects one pin for EMUX control line </toolTipHelpDescription>
    </item>
    <item name="2" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxRes,2);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var emuxValue = SCM.getIntValue(emuxRes);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;                  if (emuxmodeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                  else if (emuxValue == 2) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;&#xD;&#xA;                }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcontrolpin/1" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <localValue xsi:type="ResourceModel:StringValue" value="0"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>Selects two pins for EMUX control line</toolTipHelpDescription>
    </item>
    <item name="3" evalFunction="&#xD;&#xA;              function ForwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;                  var dep = &quot;DependentApp&quot;;&#xD;&#xA;                  var emuxctrl0Res = SCM.getResource(&quot;emuxctrl0&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl0Res,dep);&#xD;&#xA;                  var emuxctrl1Res = SCM.getResource(&quot;emuxctrl1&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl1Res,dep);&#xD;&#xA;                  var emuxctrl2Res = SCM.getResource(&quot;emuxctrl2&quot;);&#xD;&#xA;                  SCM.setStringValue(emuxctrl2Res,dep);&#xD;&#xA;&#xD;&#xA;                  if(CurrentValue == 1){&#xD;&#xA;                    SCM.setIntValue(emuxRes,3);&#xD;&#xA;                  }&#xD;&#xA;                }&#xD;&#xA;                function BackwardMapping()&#xD;&#xA;                {&#xD;&#xA;                  var emuxRes = SCM.getResource(&quot;adcgroup001_emuxcontrollinepins&quot;);&#xD;&#xA;                  var emuxValue = SCM.getIntValue(emuxRes);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;                  if (emuxmodeValue == 0) {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                  else if (emuxValue == 3) {&#xD;&#xA;                    SCM.setIntValue(currentResource,1);&#xD;&#xA;                  }else {&#xD;&#xA;                    SCM.setIntValue(currentResource,0);&#xD;&#xA;                  }&#xD;&#xA;                } " URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_erwemuxcontrolpin/2" downWardmappedList="//@consumed.23 //@consumed.24 //@consumed.25 //@consumed.21" maxValue="1" minValue="0">
      <defaultValue xsi:type="ResourceModel:IntegerValue" value="1"/>
      <localValue xsi:type="ResourceModel:StringValue" value="1"/>
      <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
      <toolTipHelpDescription>Selects three pins for EMUX control line</toolTipHelpDescription>
    </item>
  </provided>
  <provided xsi:type="ResourceModel:IntegerParameter" name="EMUX Start Select" evalFunction="&#xD;&#xA;            function ForwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxsetRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxset&quot;);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;              var CurrentValue = SCM.getIntValue(currentResource);&#xD;&#xA;              SCM.setIntValue(emuxsetRes,CurrentValue);&#xD;&#xA;            }&#xD;&#xA;            function BackwardMapping()&#xD;&#xA;            {&#xD;&#xA;              var emuxsetRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxset&quot;);&#xD;&#xA;&#xD;&#xA;                  var emuxmodeRes = SCM.getResource(&quot;adcgroupconfig/gemuxctr/emuxmode&quot;);&#xD;&#xA;                  var emuxmodeValue = SCM.getIntValue(emuxmodeRes);&#xD;&#xA;&#xD;&#xA;              var emuxsetValue = SCM.getIntValue(emuxsetRes);&#xD;&#xA;              if(emuxmodeValue==0)&#xD;&#xA;              {&#xD;&#xA;                SCM.setIntValue(currentResource,0);&#xD;&#xA;              }&#xD;&#xA;              else&#xD;&#xA;              {&#xD;&#xA;                SCM.setIntValue(currentResource,emuxsetValue);&#xD;&#xA;              }&#xD;&#xA;            }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_irwemuxset" downWardmappedList="//@consumed.54" maxValue="7" minValue="0">
    <defaultValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <localValue xsi:type="ResourceModel:StringValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <toolTipHelpDescription>This allows to define the start select for external multiplexer.</toolTipHelpDescription>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function initializes the app." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_init" APIName="ADCGROUP001_Init">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function deinitializes the app." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_deinit" APIName="ADCGROUP001_Deinit">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the group specific boundary 0." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setgroupbound0" APIName="ADCGROUP001_Start">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the group specific boundary 1." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setgroupbound1" APIName="ADCGROUP001_SetGroupBound1">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function activates the group specific request node.." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_activegroupservicerequestnode" APIName="ADCGROUP001_ActiveGroupServiceRequestNode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function Deactivates the group specific request node." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_deactivegroupservicerequestnode" APIName="ADCGROUP001_DeActiveGroupServiceRequestNode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function activates the shared service request node." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_activesharedservicerequestnode" APIName="ADCGROUP001_ActiveSharedServiceRequestNode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function Deactivates the shared service request node." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_deactivesharedservicerequestnode" APIName="ADCGROUP001_DeActiveSharedServiceRequestNode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX start selection." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxstartselection" APIName="ADCGROUP001_EMUXStartSelection">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function reads the value of EMUX start selection." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_getemuxstartselection" APIName="ADCGROUP001_GetEMUXStartSelection">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX channel select." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxchannelselect" APIName="ADCGROUP001_SetEMUXChannelSelect">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX mode." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxmode" APIName="ADCGROUP001_SetEMUXMode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX coding scheme." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxcode" APIName="ADCGROUP001_SetEMUXCode">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX sample time control." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxsampletimecontrol" APIName="ADCGROUP001_SetEMUXSampleTimeControl">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the value of EMUX channel selection type." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxchannelselectiontype" APIName="ADCGROUP001_SetEMUXChannelSelectionType">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <provided xsi:type="ResourceModel:API" description="This function sets the write control of EMUX configuration." URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_setemuxconfigurationwritecontrol" APIName="ADCGROUP001_SetEMUXConfigurationWriteControl">
    <localValue xsi:type="ResourceModel:StringValue" value="-1"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="-1"/>
  </provided>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig">
    <downWardmappedList xsi:type="ResourceModel:ResourceGroup" href="../../VADC/VADC_0.dd#//@provided.23"/>
    <requiredResource uriString="peripheral/vadc/*/group/*/config" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcglobalapp">
    <downWardmappedList xsi:type="ResourceModel:App" href="../../ADCGLOB001/1.0.28/adcglob001_0.app#/"/>
    <requiredResource uriString="app/adcglob001/*" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/temp_arbit" upWardMappingList="//@provided.10" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_requestappclass0" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_requestappclass1" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_samplingcycleclass0" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_desiredsampletimeclass0" upWardMappingList="//@provided.19 //@provided.20 //@provided.21" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_sampletimeclass0" upWardMappingList="//@provided.20" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_conversiontimeclass0" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_desiredsampletimeclass1" upWardMappingList="//@provided.23 //@provided.24 //@provided.25" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_samplingcycleclass1" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_sampletimeclass1" upWardMappingList="//@provided.24" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_conversiontimeclass1" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxsamplingcycleclass0" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxdesiredsampletimeclass0" upWardMappingList="//@provided.29 //@provided.30 //@provided.31" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxsampletimeclass0" upWardMappingList="//@provided.30" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxconversiontimeclass0" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxsamplingcycleclass1" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxdesiredsampletimeclass1" upWardMappingList="//@provided.33 //@provided.34 //@provided.35" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="43"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxsampletimeclass1" upWardMappingList="//@provided.34" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxconversiontimeclass1" isSystemDefined="true">
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroup001_emuxcontrollinepins" upWardMappingList="//@provided.44 //@provided.44/@item.0 //@provided.44/@item.1 //@provided.44/@item.2 //@provided.44/@item.3" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="3"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="3"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/temp_mast" upWardMappingList="//@provided.11" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <requiredResource uriString="" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" evalFunction="function resourceCondition() {&#xD;&#xA;        var MuxModeRes = Solver.getResource(&quot;adcgroup001_erwemuxmode/0&quot;);&#xD;&#xA;        var MuxModeVal  = Solver.getIntValue(MuxModeRes);&#xD;&#xA;&#xD;&#xA;        var emuxctrl0Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/0&quot;);&#xD;&#xA;        var emuxctrl0Val  = Solver.getIntValue(emuxctrl0Res);&#xD;&#xA;&#xD;&#xA;        if (MuxModeVal != 1) {&#xD;&#xA;          return true;&#xD;&#xA;        }&#xD;&#xA;          return false;&#xD;&#xA;        }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl0" upWardMappingList="//@provided.43 //@provided.43/@item.0 //@provided.43/@item.1 //@provided.43/@item.2 //@provided.43/@item.3 //@provided.44 //@provided.44/@item.0 //@provided.44/@item.1 //@provided.44/@item.2 //@provided.44/@item.3">
    <localValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <requiredResource uriString="app/io002/*" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" evalFunction="function resourceCondition() {&#xD;&#xA;        var MuxModeRes = Solver.getResource(&quot;adcgroup001_erwemuxmode/0&quot;);&#xD;&#xA;        var MuxModeVal  = Solver.getIntValue(MuxModeRes);&#xD;&#xA;&#xD;&#xA;        var emuxctrl0Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/0&quot;);&#xD;&#xA;        var emuxctrl0Val  = Solver.getIntValue(emuxctrl0Res);&#xD;&#xA;        var emuxctrl1Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/1&quot;);&#xD;&#xA;        var emuxctrl1Val  = Solver.getIntValue(emuxctrl1Res);&#xD;&#xA;        var emuxctrl2Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/2&quot;);&#xD;&#xA;        var emuxctrl2Val  = Solver.getIntValue(emuxctrl2Res);&#xD;&#xA;&#xD;&#xA;        if ((MuxModeVal != 1) &amp;&amp; ((emuxctrl1Val == 1) || (emuxctrl2Val == 1))) {&#xD;&#xA;          return true;&#xD;&#xA;        }&#xD;&#xA;          return false;&#xD;&#xA;        }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl1" upWardMappingList="//@provided.43/@item.0 //@provided.43/@item.1 //@provided.43/@item.2 //@provided.43/@item.3 //@provided.44 //@provided.44/@item.0 //@provided.44/@item.1 //@provided.44/@item.2 //@provided.44/@item.3">
    <localValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <requiredResource uriString="app/io002/*" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" evalFunction="function resourceCondition() {&#xD;&#xA;        var MuxModeRes = Solver.getResource(&quot;adcgroup001_erwemuxmode/0&quot;);&#xD;&#xA;        var MuxModeVal  = Solver.getIntValue(MuxModeRes);&#xD;&#xA;&#xD;&#xA;        var emuxctrl0Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/0&quot;);&#xD;&#xA;        var emuxctrl0Val  = Solver.getIntValue(emuxctrl0Res);&#xD;&#xA;        var emuxctrl1Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/1&quot;);&#xD;&#xA;        var emuxctrl1Val  = Solver.getIntValue(emuxctrl1Res);&#xD;&#xA;        var emuxctrl2Res = Solver.getResource(&quot;adcgroup001_erwemuxcontrolpin/2&quot;);&#xD;&#xA;        var emuxctrl2Val  = Solver.getIntValue(emuxctrl2Res);&#xD;&#xA;&#xD;&#xA;        if ((MuxModeVal != 1) &amp;&amp; (emuxctrl2Val == 1))  {&#xD;&#xA;          return true;&#xD;&#xA;        }&#xD;&#xA;          return false;&#xD;&#xA;        }" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl2" upWardMappingList="//@provided.43/@item.0 //@provided.43/@item.1 //@provided.43/@item.2 //@provided.43/@item.3 //@provided.44 //@provided.44/@item.0 //@provided.44/@item.1 //@provided.44/@item.2 //@provided.44/@item.3">
    <localValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <globalValue xsi:type="ResourceModel:StringValue" value="DependentApp"/>
    <requiredResource uriString="app/io002/*" uriType="LOCALTYPE"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gsample" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.24"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gqueuereq" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.25"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/emux2" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.26"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/emux0" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.27"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/emux1" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.28"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/garbcnt" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.29"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gsyncreq" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.30"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gscanreq" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.31"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/globcfg_dpcal" upWardMappingList="//@provided.21 //@provided.25 //@provided.31 //@provided.35 //@provided.37 //@provided.37/@item.0" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.32"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/garbcfg" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.33"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/garbcfg/arbm" upWardMappingList="//@provided.38 //@provided.38/@item.0" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.33/@bitFields.2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/garbcfg/arbrnd" upWardMappingList="//@provided.41 //@provided.41/@item.0" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.33/@bitFields.1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass0" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.34"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass0/stcs" upWardMappingList="//@provided.19 //@provided.22" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.34/@bitFields.0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass0/cms" upWardMappingList="//@provided.12 //@provided.12/@item.0 //@provided.21" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.34/@bitFields.1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass0/stce" upWardMappingList="//@provided.29 //@provided.32" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.34/@bitFields.2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass0/cme" upWardMappingList="//@provided.27 //@provided.27/@item.0 //@provided.31" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.34/@bitFields.3"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass1" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.35"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass1/stcs" upWardMappingList="//@provided.23 //@provided.26" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.35/@bitFields.0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass1/cms" upWardMappingList="//@provided.13 //@provided.13/@item.0 //@provided.25" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.35/@bitFields.1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass1/stce" upWardMappingList="//@provided.33 //@provided.36" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.35/@bitFields.2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/giclass1/cme" upWardMappingList="//@provided.28 //@provided.28/@item.0 //@provided.35" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.35/@bitFields.3"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gbound" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.36"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gbound/boundary0" upWardMappingList="//@provided.39" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.36/@bitFields.0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gbound/boundary1" upWardMappingList="//@provided.40" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.36/@bitFields.1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gemuxctr" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.37"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gemuxctr/emxcod" upWardMappingList="//@provided.42 //@provided.42/@item.0" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.37/@bitFields.4"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gemuxctr/emuxmode" upWardMappingList="//@provided.43 //@provided.43/@item.0 //@provided.43/@item.1 //@provided.43/@item.2 //@provided.43/@item.3 //@provided.44 //@provided.44/@item.0 //@provided.44/@item.1 //@provided.44/@item.2 //@provided.44/@item.3 //@provided.45" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.37/@bitFields.3"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gemuxctr/emuxset" upWardMappingList="//@provided.45" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:BitField" href="../../VADC/VADC_0.dd#//@provided.37/@bitFields.0"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/gsract" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
    <downWardmappedList xsi:type="ResourceModel:RegisterParameter" href="../../VADC/VADC_0.dd#//@provided.38"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="00000000"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcglobalapp/adcglob001_irwperibusclock" upWardMappingList="//@provided.16 //@provided.21 //@provided.25 //@provided.31 //@provided.35" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:StringValue" value="2"/>
    <downWardmappedList xsi:type="ResourceModel:IntegerParameter" href="../../ADCGLOB001/1.0.28/adcglob001_0.app#//@provided.4"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="78"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcglobalapp/adcglob001_irwselanalogclock" upWardMappingList="//@provided.18 //@provided.19 //@provided.20 //@provided.21 //@provided.22 //@provided.23 //@provided.24 //@provided.25 //@provided.26 //@provided.29 //@provided.30 //@provided.31 //@provided.32 //@provided.33 //@provided.34 //@provided.35 //@provided.36" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:StringValue" value="1E"/>
    <downWardmappedList xsi:type="ResourceModel:IntegerParameter" href="../../ADCGLOB001/1.0.28/adcglob001_0.app#//@provided.11"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="1e"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcglobalapp/adcglob001_erwdblclockmsb" upWardMappingList="//@provided.17 //@provided.21 //@provided.25 //@provided.31 //@provided.35" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="0"/>
    <downWardmappedList xsi:type="ResourceModel:EnumerationParameter" href="../../ADCGLOB001/1.0.28/adcglob001_0.app#//@provided.2"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="0"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl0/io002_erwoutputmode" isSystemDefined="true"/>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl0/io002_erwoutputmode/0" upWardMappingList="//@provided.43" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl1/io002_erwoutputmode" isSystemDefined="true"/>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl1/io002_erwoutputmode/0" upWardMappingList="//@provided.43" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl2/io002_erwoutputmode" isSystemDefined="true"/>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/emuxctrl2/io002_erwoutputmode/0" upWardMappingList="//@provided.43" isSystemDefined="true">
    <localValue xsi:type="ResourceModel:IntegerValue" value="1"/>
    <globalValue xsi:type="ResourceModel:IntegerValue" value="1"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/global_signal" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.39"/>
  </consumed>
  <consumed xsi:type="ResourceModel:ProxyResource" URI="http://www.infineon.com/1.0.28/app/adcgroup001/0/adcgroupconfig/select" isSystemDefined="true">
    <downWardmappedList xsi:type="ResourceModel:SignalDeclaration" href="../../VADC/VADC_0.dd#//@provided.40"/>
  </consumed>
  <propertyConstants name="uriDevice" value="http://www.infineon.com/Device/0.1.28">
    <propertyConstants name="uriPeri_App" value="http://www.infineon.com/0.1.28/peripheral/adc/0"/>
  </propertyConstants>
  <categoryDescription description="Peripheral Specific Apps" name="Peripheral Specific Apps">
    <subCategory description="Analog" name="Analog">
      <subCategory description="Analog to Digital Converter" name="Versatile Analog to Digital Converter(VADC)"/>
    </subCategory>
  </categoryDescription>
  <categoryDescription description="Basic Applications" name="Basic Applications">
    <subCategory description="Acquisition of Analog Signals" name="Acquisition of Analog Signals"/>
  </categoryDescription>
  <manifestInfo version="1.0.28">
    <keywords>ADC Group</keywords>
    <keywords>Group input class</keywords>
    <keywords>Input class</keywords>
    <keywords>VADC</keywords>
    <keywords>ADC</keywords>
    <keywords>EMUX</keywords>
    <properties xsi:type="ResourceModel:AppProperties_1_0" initProvider="true"/>
  </manifestInfo>
  <datagenerate fileName="ADCGroup001.h" fileType="HFILE" templateFileName="ADCGroup001.hdt" templateEngine="NONE" fileAction="COPY"/>
  <datagenerate fileName="ADCGroup001_Conf.h" fileType="HFILE" templateFileName="ADCGroup001_Confh.jet"/>
  <datagenerate fileName="ADCGroup001_Conf.c" templateFileName="ADCGroup001_Confc.jet"/>
  <datagenerate fileName="ADCGroup001.c" templateFileName="ADCGroup001c.jet"/>
  <connections sourceSignal="app/adcgroup001/0/adcgroup001_emux0_vs" targetSignal="emuxctrl0/io002_vs" constraintFunction=""/>
  <connections sourceSignal="app/adcgroup001/0/adcgroup001_emux1_vs" targetSignal="emuxctrl1/io002_vs" constraintFunction=""/>
  <connections sourceSignal="app/adcgroup001/0/adcgroup001_emux2_vs" targetSignal="emuxctrl2/io002_vs" constraintFunction=""/>
  <softwareIdList>45.0.1</softwareIdList>
  <softwareIdList>45.0.2</softwareIdList>
  <softwareIdList>45.0.3</softwareIdList>
  <softwareIdList>45.2.3</softwareIdList>
  <softwareIdList>45.4.2</softwareIdList>
  <softwareIdList>45.4.3</softwareIdList>
  <softwareIdList>45.0200.1</softwareIdList>
  <softwareIdList>45.0200.2</softwareIdList>
  <softwareIdList>45.0200.3</softwareIdList>
  <softwareIdList>45.0202.3</softwareIdList>
  <softwareIdList>45.0204.3</softwareIdList>
  <softwareIdList>45.0204.2</softwareIdList>
  <softwareIdList>44.0.3</softwareIdList>
  <softwareIdList>44.00.5</softwareIdList>
  <softwareIdList>44.02.3</softwareIdList>
  <softwareIdList>44.02.5</softwareIdList>
  <softwareIdList>42.00.5</softwareIdList>
  <softwareIdList>42.0.6</softwareIdList>
  <softwareIdList>42.04.5</softwareIdList>
  <softwareIdList>42.04.6</softwareIdList>
  <softwareIdList>12.0100.7</softwareIdList>
  <softwareIdList>13.0102.7</softwareIdList>
  <softwareIdList>12.0101.7</softwareIdList>
  <softwareIdList>12.0102.9</softwareIdList>
  <softwareIdList>12.0102.8</softwareIdList>
  <softwareIdList>13.0101.9</softwareIdList>
  <softwareIdList>13.0101.7</softwareIdList>
  <softwareIdList>42.0200.5</softwareIdList>
  <softwareIdList>42.0204.5</softwareIdList>
  <softwareIdList>42.0204.6</softwareIdList>
  <softwareIdList>42.0200.6</softwareIdList>
  <softwareIdList>42.0208.6</softwareIdList>
  <softwareIdList>44.0200.5</softwareIdList>
  <softwareIdList>44.0202.3</softwareIdList>
  <softwareIdList>44.0202.5</softwareIdList>
  <softwareIdList>44.0200.3</softwareIdList>
  <softwareIdList>45.0300.1</softwareIdList>
  <softwareIdList>45.0300.3</softwareIdList>
  <softwareIdList>45.0302.3</softwareIdList>
  <softwareIdList>45.0304.2</softwareIdList>
  <softwareIdList>45.0300.2</softwareIdList>
  <softwareIdList>45.0304.3</softwareIdList>
  <softwareIdList>12.0101.11</softwareIdList>
  <softwareIdList>12.0102.10</softwareIdList>
  <softwareIdList>12.0102.11</softwareIdList>
  <softwareIdList>13.0101.10</softwareIdList>
  <softwareIdList>13.0101.11</softwareIdList>
  <softwareIdList>13.0102.10</softwareIdList>
  <softwareIdList>13.0102.11</softwareIdList>
  <softwareIdList>13.0102.9</softwareIdList>
</ResourceModel:App>
