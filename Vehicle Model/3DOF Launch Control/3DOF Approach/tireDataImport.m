orig = load( ...
  'C:\Users\nhola\OneDrive\Documents\GitHub\Tire-Data\Models\Hoosier_R25B_16x75-10x7.mat' ...
);

cleanT = rmfield(orig.Tire, 'Source');
Pressure    = 70;    
Inclination = 1;      
Model       = struct('Pure','Pacejka','Combined','MNC');
Idx         = 1;     % rear‐axle index

outPath = ['Hoosier_R25B_clean.mat'];

save(outPath, 'cleanT','Pressure','Inclination','Model','Idx')
