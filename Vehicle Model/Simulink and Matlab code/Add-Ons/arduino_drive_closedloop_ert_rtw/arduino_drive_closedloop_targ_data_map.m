  function targMap = targDataMap(),

  ;%***********************
  ;% Create Parameter Map *
  ;%***********************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 7;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc paramMap
    ;%
    paramMap.nSections           = nTotSects;
    paramMap.sectIdxOffset       = sectIdxOffset;
      paramMap.sections(nTotSects) = dumSection; %prealloc
    paramMap.nTotData            = -1;
    
    ;%
    ;% Auto data (mlgofdcvbcq)
    ;%
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.PWM_pinNumber
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.PWM_pinNumber_auhtru3tsq
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(1) = section;
      clear section
      
      section.nData     = 12;
      section.data(12)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.DigitalInput_SampleTime
	  section.data(1).logicalSrcIdx = 2;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.DigitalInput_SampleT_aqt1x4pe0z
	  section.data(2).logicalSrcIdx = 3;
	  section.data(2).dtTransOffset = 1;
	
	  ;% mlgofdcvbcq.uDLookupTable_tableData
	  section.data(3).logicalSrcIdx = 4;
	  section.data(3).dtTransOffset = 2;
	
	  ;% mlgofdcvbcq.uDLookupTable_bp01Data
	  section.data(4).logicalSrcIdx = 5;
	  section.data(4).dtTransOffset = 5;
	
	  ;% mlgofdcvbcq.RandomNumber1_Mean
	  section.data(5).logicalSrcIdx = 6;
	  section.data(5).dtTransOffset = 8;
	
	  ;% mlgofdcvbcq.RandomNumber1_StdDev
	  section.data(6).logicalSrcIdx = 7;
	  section.data(6).dtTransOffset = 9;
	
	  ;% mlgofdcvbcq.RandomNumber1_Seed
	  section.data(7).logicalSrcIdx = 8;
	  section.data(7).dtTransOffset = 10;
	
	  ;% mlgofdcvbcq.uDLookupTable_tableD_nmmn3m3q23
	  section.data(8).logicalSrcIdx = 9;
	  section.data(8).dtTransOffset = 11;
	
	  ;% mlgofdcvbcq.uDLookupTable_bp01Da_ko0fknrlx3
	  section.data(9).logicalSrcIdx = 10;
	  section.data(9).dtTransOffset = 14;
	
	  ;% mlgofdcvbcq.RandomNumber1_Mean_ccqmdi0cl2
	  section.data(10).logicalSrcIdx = 11;
	  section.data(10).dtTransOffset = 17;
	
	  ;% mlgofdcvbcq.RandomNumber1_StdDev_kl5vevfdhz
	  section.data(11).logicalSrcIdx = 12;
	  section.data(11).dtTransOffset = 18;
	
	  ;% mlgofdcvbcq.RandomNumber1_Seed_ii5ygsmxhx
	  section.data(12).logicalSrcIdx = 13;
	  section.data(12).dtTransOffset = 19;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(2) = section;
      clear section
      
      section.nData     = 19;
      section.data(19)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.Gain_Gain
	  section.data(1).logicalSrcIdx = 14;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.Constant2_Value
	  section.data(2).logicalSrcIdx = 15;
	  section.data(2).dtTransOffset = 1;
	
	  ;% mlgofdcvbcq.Speed_Value
	  section.data(3).logicalSrcIdx = 16;
	  section.data(3).dtTransOffset = 2;
	
	  ;% mlgofdcvbcq.TurnAngle_Value
	  section.data(4).logicalSrcIdx = 17;
	  section.data(4).dtTransOffset = 3;
	
	  ;% mlgofdcvbcq.Saturation_UpperSat
	  section.data(5).logicalSrcIdx = 18;
	  section.data(5).dtTransOffset = 4;
	
	  ;% mlgofdcvbcq.Saturation_LowerSat
	  section.data(6).logicalSrcIdx = 19;
	  section.data(6).dtTransOffset = 5;
	
	  ;% mlgofdcvbcq.Scale_Gain
	  section.data(7).logicalSrcIdx = 20;
	  section.data(7).dtTransOffset = 6;
	
	  ;% mlgofdcvbcq.TmpLatchAtTriggeredSubsystemInp
	  section.data(8).logicalSrcIdx = 21;
	  section.data(8).dtTransOffset = 7;
	
	  ;% mlgofdcvbcq.Constant_Value
	  section.data(9).logicalSrcIdx = 22;
	  section.data(9).dtTransOffset = 8;
	
	  ;% mlgofdcvbcq.Gain_Gain_nlnczbyk1z
	  section.data(10).logicalSrcIdx = 23;
	  section.data(10).dtTransOffset = 9;
	
	  ;% mlgofdcvbcq.Saturation1_UpperSat
	  section.data(11).logicalSrcIdx = 24;
	  section.data(11).dtTransOffset = 10;
	
	  ;% mlgofdcvbcq.Saturation1_LowerSat
	  section.data(12).logicalSrcIdx = 25;
	  section.data(12).dtTransOffset = 11;
	
	  ;% mlgofdcvbcq.Scale1_Gain
	  section.data(13).logicalSrcIdx = 26;
	  section.data(13).dtTransOffset = 12;
	
	  ;% mlgofdcvbcq.Constant_Value_hlghp14ktg
	  section.data(14).logicalSrcIdx = 27;
	  section.data(14).dtTransOffset = 13;
	
	  ;% mlgofdcvbcq.Constant1_Value
	  section.data(15).logicalSrcIdx = 28;
	  section.data(15).dtTransOffset = 14;
	
	  ;% mlgofdcvbcq.Memory_X0
	  section.data(16).logicalSrcIdx = 29;
	  section.data(16).dtTransOffset = 15;
	
	  ;% mlgofdcvbcq.WeightedSampleTime_WtEt
	  section.data(17).logicalSrcIdx = 30;
	  section.data(17).dtTransOffset = 16;
	
	  ;% mlgofdcvbcq.Memory_X0_edgfrcrw21
	  section.data(18).logicalSrcIdx = 31;
	  section.data(18).dtTransOffset = 17;
	
	  ;% mlgofdcvbcq.WeightedSampleTime_W_j5p1gp5d2w
	  section.data(19).logicalSrcIdx = 32;
	  section.data(19).dtTransOffset = 18;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(3) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.UnitDelay_InitialCondition
	  section.data(1).logicalSrcIdx = 33;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.UnitDelay1_InitialCondition
	  section.data(2).logicalSrcIdx = 34;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(4) = section;
      clear section
      
      section.nData     = 5;
      section.data(5)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.SwitchControl_Value
	  section.data(1).logicalSrcIdx = 35;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.Constant_Value_jzepk1bct5
	  section.data(2).logicalSrcIdx = 36;
	  section.data(2).dtTransOffset = 1;
	
	  ;% mlgofdcvbcq.Constant1_Value_j0uh2b3d3w
	  section.data(3).logicalSrcIdx = 37;
	  section.data(3).dtTransOffset = 2;
	
	  ;% mlgofdcvbcq.Constant2_Value_ohnaqb1c5m
	  section.data(4).logicalSrcIdx = 38;
	  section.data(4).dtTransOffset = 3;
	
	  ;% mlgofdcvbcq.Constant3_Value
	  section.data(5).logicalSrcIdx = 39;
	  section.data(5).dtTransOffset = 4;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(5) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.jlkzq3g5eo.UnitDelay_InitialCondition
	  section.data(1).logicalSrcIdx = 40;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.jlkzq3g5eo.Constant_Value
	  section.data(2).logicalSrcIdx = 41;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(6) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% mlgofdcvbcq.jvdzn1pv5gv.UnitDelay_InitialCondition
	  section.data(1).logicalSrcIdx = 42;
	  section.data(1).dtTransOffset = 0;
	
	  ;% mlgofdcvbcq.jvdzn1pv5gv.Constant_Value
	  section.data(2).logicalSrcIdx = 43;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      paramMap.sections(7) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (parameter)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    paramMap.nTotData = nTotData;
    


  ;%**************************
  ;% Create Block Output Map *
  ;%**************************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 4;
    sectIdxOffset = 0;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc sigMap
    ;%
    sigMap.nSections           = nTotSects;
    sigMap.sectIdxOffset       = sectIdxOffset;
      sigMap.sections(nTotSects) = dumSection; %prealloc
    sigMap.nTotData            = -1;
    
    ;%
    ;% Auto data (k0udfhehnym)
    ;%
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% k0udfhehnym.fxuvwpclnn
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% k0udfhehnym.h5ukskkfbg
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(1) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% k0udfhehnym.bpu32hkls3
	  section.data(1).logicalSrcIdx = 2;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(2) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% k0udfhehnym.jlkzq3g5eo.nffsoxy2nc
	  section.data(1).logicalSrcIdx = 3;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(3) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% k0udfhehnym.jvdzn1pv5gv.nffsoxy2nc
	  section.data(1).logicalSrcIdx = 4;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      sigMap.sections(4) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (signal)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    sigMap.nTotData = nTotData;
    


  ;%*******************
  ;% Create DWork Map *
  ;%*******************
      
    nTotData      = 0; %add to this count as we go
    nTotSects     = 13;
    sectIdxOffset = 4;
    
    ;%
    ;% Define dummy sections & preallocate arrays
    ;%
    dumSection.nData = -1;  
    dumSection.data  = [];
    
    dumData.logicalSrcIdx = -1;
    dumData.dtTransOffset = -1;
    
    ;%
    ;% Init/prealloc dworkMap
    ;%
    dworkMap.nSections           = nTotSects;
    dworkMap.sectIdxOffset       = sectIdxOffset;
      dworkMap.sections(nTotSects) = dumSection; %prealloc
    dworkMap.nTotData            = -1;
    
    ;%
    ;% Auto data (p400cl0h3bv)
    ;%
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.i33r30rat1
	  section.data(1).logicalSrcIdx = 0;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.dnb5ihzdv3
	  section.data(2).logicalSrcIdx = 1;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(1) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.dddwrko252
	  section.data(1).logicalSrcIdx = 2;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.dbvjy0neef
	  section.data(2).logicalSrcIdx = 3;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(2) = section;
      clear section
      
      section.nData     = 8;
      section.data(8)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.puuun0g1ka.LoggedData
	  section.data(1).logicalSrcIdx = 4;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.ft3yganqyg.LoggedData
	  section.data(2).logicalSrcIdx = 5;
	  section.data(2).dtTransOffset = 1;
	
	  ;% p400cl0h3bv.o4pmjogijr
	  section.data(3).logicalSrcIdx = 6;
	  section.data(3).dtTransOffset = 2;
	
	  ;% p400cl0h3bv.f4squnte5k
	  section.data(4).logicalSrcIdx = 7;
	  section.data(4).dtTransOffset = 3;
	
	  ;% p400cl0h3bv.hjpqwgvm3b
	  section.data(5).logicalSrcIdx = 8;
	  section.data(5).dtTransOffset = 4;
	
	  ;% p400cl0h3bv.khph4ikbkb
	  section.data(6).logicalSrcIdx = 9;
	  section.data(6).dtTransOffset = 5;
	
	  ;% p400cl0h3bv.b210yie0cc
	  section.data(7).logicalSrcIdx = 10;
	  section.data(7).dtTransOffset = 6;
	
	  ;% p400cl0h3bv.gjsxohepqr
	  section.data(8).logicalSrcIdx = 11;
	  section.data(8).dtTransOffset = 7;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(3) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.mr3gvfkvey
	  section.data(1).logicalSrcIdx = 12;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.bhzvf3tef0
	  section.data(2).logicalSrcIdx = 13;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(4) = section;
      clear section
      
      section.nData     = 3;
      section.data(3)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.b1fsw2px41
	  section.data(1).logicalSrcIdx = 14;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.pz2fu52rhj
	  section.data(2).logicalSrcIdx = 15;
	  section.data(2).dtTransOffset = 1;
	
	  ;% p400cl0h3bv.otbi31lqu3
	  section.data(3).logicalSrcIdx = 16;
	  section.data(3).dtTransOffset = 2;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(5) = section;
      clear section
      
      section.nData     = 2;
      section.data(2)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.olaburdj1c
	  section.data(1).logicalSrcIdx = 17;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.omvhfz20ws
	  section.data(2).logicalSrcIdx = 18;
	  section.data(2).dtTransOffset = 1;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(6) = section;
      clear section
      
      section.nData     = 4;
      section.data(4)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.mjkyk2odqm
	  section.data(1).logicalSrcIdx = 19;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.l5pnz1izkh
	  section.data(2).logicalSrcIdx = 20;
	  section.data(2).dtTransOffset = 1;
	
	  ;% p400cl0h3bv.eapvmglwjn
	  section.data(3).logicalSrcIdx = 21;
	  section.data(3).dtTransOffset = 2;
	
	  ;% p400cl0h3bv.f2b5ktuofo
	  section.data(4).logicalSrcIdx = 22;
	  section.data(4).dtTransOffset = 3;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(7) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.lwbfzc5nm2
	  section.data(1).logicalSrcIdx = 23;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(8) = section;
      clear section
      
      section.nData     = 6;
      section.data(6)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.hktxzivzqr
	  section.data(1).logicalSrcIdx = 24;
	  section.data(1).dtTransOffset = 0;
	
	  ;% p400cl0h3bv.ifxuvcbeaw
	  section.data(2).logicalSrcIdx = 25;
	  section.data(2).dtTransOffset = 1;
	
	  ;% p400cl0h3bv.olpb4ybrlz
	  section.data(3).logicalSrcIdx = 26;
	  section.data(3).dtTransOffset = 2;
	
	  ;% p400cl0h3bv.h5ovvlypwj
	  section.data(4).logicalSrcIdx = 27;
	  section.data(4).dtTransOffset = 3;
	
	  ;% p400cl0h3bv.euotdkc3wl
	  section.data(5).logicalSrcIdx = 28;
	  section.data(5).dtTransOffset = 4;
	
	  ;% p400cl0h3bv.j3qowqb4a2
	  section.data(6).logicalSrcIdx = 29;
	  section.data(6).dtTransOffset = 5;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(9) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.jlkzq3g5eo.pkr0q02r5i
	  section.data(1).logicalSrcIdx = 30;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(10) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.jlkzq3g5eo.kvf5dhtmni
	  section.data(1).logicalSrcIdx = 31;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(11) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.jvdzn1pv5gv.pkr0q02r5i
	  section.data(1).logicalSrcIdx = 32;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(12) = section;
      clear section
      
      section.nData     = 1;
      section.data(1)  = dumData; %prealloc
      
	  ;% p400cl0h3bv.jvdzn1pv5gv.kvf5dhtmni
	  section.data(1).logicalSrcIdx = 33;
	  section.data(1).dtTransOffset = 0;
	
      nTotData = nTotData + section.nData;
      dworkMap.sections(13) = section;
      clear section
      
    
      ;%
      ;% Non-auto Data (dwork)
      ;%
    

    ;%
    ;% Add final counts to struct.
    ;%
    dworkMap.nTotData = nTotData;
    


  ;%
  ;% Add individual maps to base struct.
  ;%

  targMap.paramMap  = paramMap;    
  targMap.signalMap = sigMap;
  targMap.dworkMap  = dworkMap;
  
  ;%
  ;% Add checksums to base struct.
  ;%


  targMap.checksum0 = 378787120;
  targMap.checksum1 = 874151987;
  targMap.checksum2 = 3342208301;
  targMap.checksum3 = 245364475;

