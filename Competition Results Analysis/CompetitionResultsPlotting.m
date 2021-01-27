clc; clear; close all;

%% Competition Results Plotting
% This script plots the results from previous competitions dynamic events
% by loading the data from the "Dynamic Event Score Compilation"
% spreadsheet in the Literature & Reference -> Competition folder.
%
% Blake Christierson - bechristierson@ucdavis.edu 

%% Loading Data from Spreadsheet
[Acceleration, Skidpad] = SpreadsheetImport();

%% Plotting
Style  = {'-'    , '--'  , ':'   , '-.'  , '-', '-', '-', '-', '-'};
Marker = {'none' , 'none', 'none', 'none', 'x', '+', 'o', 's', 'd'}; 
Color  = colormap('parula'); close all;

Comp = unique({Acceleration.Competition});
Year = unique([Acceleration.Year]);

Color = Color( floor( linspace(50,size(Color,1)-50,length(Year)) ), : );

LegendText = {' '};
h = [];
for i = 1:length(Comp) 
    for j = 1:length(Year) 
        AccelData = Acceleration( strcmp({Acceleration.Competition}, Comp(i)) & ...
            [Acceleration.Year] == Year(j) );
        
        SkidpadData = Skidpad( strcmp({Skidpad.Competition}, Comp(i)) & ...
            [Skidpad.Year] == Year(j) );
        
        subplot(2,1,1)
        if isempty( h )
            h(1) = plot( [AccelData.Place], [AccelData.Time], ...
                'LineStyle', Style{i}, 'Marker', Marker{i}, 'Color', Color(j,:) ); hold on; 
        else
            h(end+1) = plot( [AccelData.Place], [AccelData.Time], ...
                'LineStyle', Style{i}, 'Marker', Marker{i}, 'Color', Color(j,:) ); hold on; 
        end
        
        scatter( [AccelData( strcmpi( {AccelData.Team}, 'Univ of Calif - Davis' ) ).Place], ...
            [AccelData( strcmpi( {AccelData.Team}, 'Univ of Calif - Davis' ) ).Time], ...
            [], Color(j,:), 'Marker', 'p', 'MarkerFaceColor', 'flat' ); hold on;
        
        subplot(2,1,2)
        plot( [SkidpadData.Place], [SkidpadData.Time], ...
            'LineStyle', Style{i}, 'Marker', Marker{i}, 'Color', Color(j,:) ); hold on;
        
        scatter( [SkidpadData( strcmpi( {SkidpadData.Team}, 'Univ of Calif - Davis' ) ).Place], ...
            [SkidpadData( strcmpi( {SkidpadData.Team}, 'Univ of Calif - Davis' ) ).Time], ...
            [], Color(j,:), 'Marker', 'p', 'MarkerFaceColor', 'flat' ); hold on;
        
        LegendText{end+1} = [Comp{i}, ' ', num2str(Year(j))];
    end
end

h(end+1) = scatter( 20, 100, [], 'k', 'Marker', 'p', 'MarkerFaceColor', 'flat' );
LegendText{end+1} = 'FRUCD';

subplot(2,1,1)
title( 'Acceleration Event' )
xlabel( 'Competition Place' )
ylabel( 'Time [s]')

xlim( [1, max( [Acceleration.Place] )] )
ylim( [0.9*min( [Acceleration.Time] ), 1.05*max( [Acceleration.Time] )] )

legend( h, LegendText(2:end) )

subplot(2,1,2)
title( 'Skidpad Event' )
xlabel( 'Competition Place' )
ylabel( 'Time [s]')

xlim( [1, max( [Skidpad.Place] )] )
ylim( [0.9*min( [Skidpad.Time] ), 1.05*max( [Skidpad.Time] )] )

%% Local Functions
function [Acceleration, Skidpad] = SpreadsheetImport()
    % Import Spreadsheets
    Spreadsheet.Acceleration = GetGoogleSpreadsheet( ...
        '1m95DDU05gu4s2VLQ166Xz5h6PHz7ItxepBbzQkxlHEs', 'Acceleration' );
    
    Spreadsheet.Skidpad = GetGoogleSpreadsheet( ...
        '1m95DDU05gu4s2VLQ166Xz5h6PHz7ItxepBbzQkxlHEs', 'Skidpad' );
    
    Spreadsheet.Autocross = GetGoogleSpreadsheet( ...
        '1m95DDU05gu4s2VLQ166Xz5h6PHz7ItxepBbzQkxlHEs', 'Autocross' );

    Spreadsheet.Endurance = GetGoogleSpreadsheet( ...
        '1m95DDU05gu4s2VLQ166Xz5h6PHz7ItxepBbzQkxlHEs', 'Endurance' );
    
    % Allocate Acceleration Structure
    Acceleration = struct();
    Acceleration( size(Spreadsheet.Acceleration, 1) - 3 ).Competition = [];
    for i = 2 : size(Spreadsheet.Acceleration, 1)
        Acceleration(i-1).Competition = Spreadsheet.Acceleration{i,1};
        Acceleration(i-1).Year        = str2double(Spreadsheet.Acceleration{i,2});
        Acceleration(i-1).Team        = Spreadsheet.Acceleration{i,3};
        Acceleration(i-1).Number      = str2double(Spreadsheet.Acceleration{i,4});
        
        Acceleration(i-1).Time   = str2double(Spreadsheet.Acceleration{i,5});
        Acceleration(i-1).Place  = str2double(Spreadsheet.Acceleration{i,6});
        Acceleration(i-1).Points = str2double(Spreadsheet.Acceleration{i,7});
        
        Acceleration(i-1).Mean   = str2double(Spreadsheet.Acceleration{i,8});
        Acceleration(i-1).PerDev = str2double(Spreadsheet.Acceleration{i,9}(1:end-1));
    end
    
    % Allocate Skidpad Structure
    Skidpad = struct();
    Skidpad( size(Spreadsheet.Skidpad, 1) - 3 ).Competition = [];
    for i = 2 : size(Spreadsheet.Skidpad, 1)
        Skidpad(i-1).Competition = Spreadsheet.Skidpad{i,1};
        Skidpad(i-1).Year        = str2double(Spreadsheet.Skidpad{i,2});
        Skidpad(i-1).Team        = Spreadsheet.Skidpad{i,3};
        Skidpad(i-1).Number      = str2double(Spreadsheet.Skidpad{i,4});
        
        Skidpad(i-1).Time   = str2double(Spreadsheet.Skidpad{i,5});
        Skidpad(i-1).Place  = str2double(Spreadsheet.Skidpad{i,6});
        Skidpad(i-1).Points = str2double(Spreadsheet.Skidpad{i,7});
        
        Skidpad(i-1).Mean   = str2double(Spreadsheet.Skidpad{i,8});
        Skidpad(i-1).PerDev = str2double(Spreadsheet.Skidpad{i,9}(1:end-1));
    end
    
    %%% Local Spreadsheet Import Function
    function Spreadsheet = GetGoogleSpreadsheet(WorkbookID, SheetName)
        % Download a google spreadsheet as csv and import into a Matlab cell array.
        %
        % [DOCID] see the value after 'key=' in your spreadsheet's url
        %           e.g. '0AmQ013fj5234gSXFAWLK1REgwRW02hsd3c'
        %
        % [result] cell array of the the values in the spreadsheet
        %
        % IMPORTANT: The spreadsheet must be shared with the "anyone with the link" option
        %
        % This has no error handling and has not been extensively tested.
        % Please report issues on Matlab FX.
        %
        % DM, Jan 2013

        % https://docs.google.com/spreadsheets/d/{key}/gviz/tq?tqx=out:csv&sheet={sheet_name}

        LoginURL = 'https://www.google.com'; 
        CSVURL = ['https://docs.google.com/spreadsheets/d/', WorkbookID              , ...
                  '/gviz/tq?tqx=out:csv&sheet='            , strrep(SheetName,' ','+') ];

        %Step 1: Go to google.com to collect some cookies
        CookieManager = java.net.CookieManager([], java.net.CookiePolicy.ACCEPT_ALL);
        java.net.CookieHandler.setDefault(CookieManager);
        Handler = sun.net.www.protocol.https.Handler;
        Connection = java.net.URL([],LoginURL,Handler).openConnection();
        Connection.getInputStream();

        %Step 2: Go to the spreadsheet export url and download the csv
        Connection2 = java.net.URL([],CSVURL,Handler).openConnection();
        Spreadsheet = Connection2.getInputStream();
        Spreadsheet = char(ReadStream(Spreadsheet));

        %Step 3: Convert the csv to a cell array
        Spreadsheet = ParseCSV(Spreadsheet);

        % Local Functions
        function out = ReadStream(inStream)
        %READSTREAM Read all bytes from stream to uint8
        %From: http://stackoverflow.com/a/1323535
            import com.mathworks.mlwidgets.io.InterruptibleStreamCopier;
            byteStream = java.io.ByteArrayOutputStream();
            isc = InterruptibleStreamCopier.getInterruptibleStreamCopier();
            isc.copyStream(inStream, byteStream);
            inStream.close();
            byteStream.close();
            out = typecast(byteStream.toByteArray', 'uint8'); 
        end
        function data = ParseCSV(data)
        % Splits data into individual lines
            data = textscan(data,'%s','whitespace','\n');
            data = data{1};
            for ii=1:length(data)
               % For each line, split the string into its comma-delimited units
               % The '%q' format deals with the "quoting" convention appropriately.
               tmp = textscan(data{ii},'%q','delimiter',',');
               data(ii,1:length(tmp{1})) = tmp{1};
            end
        end
    end
end