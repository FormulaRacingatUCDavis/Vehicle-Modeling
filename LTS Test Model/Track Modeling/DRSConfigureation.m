clear; clc; close all;

function main(track)
    % Load track
    TrackInfo = load(track).TrackInfo;
    TrackInfo.drs = zeros(length(TrackInfo.x), 1);
    
    % create the GUI
    % figure and lines
    fig = figure();
    ax = axes('Parent', fig, 'Position', [0.08 0.25 0.88 0.70]);
    hold(ax, "on")
    plot(ax, TrapckInfo.coords(:, 1), TrackInfo.coords(:, 2));
    drs_section = plot(ax, 0, 0, "r");
    
    % GUI parts
    start_point = uicontrol('Parent', fig, 'Style','slider', ...
                            'Units','normalized', 'Position',[0.08 0.12 0.70 0.06], ...
                            'Min',1, 'Max',length(TrackInfo.coords), "Value",1);
    
    end_point = uicontrol('Parent', fig, 'Style','slider', ...
                            'Units','normalized', 'Position',[0.08 0.05 0.70 0.06], ...
                            'Min',1, 'Max',length(TrackInfo.coords), "Value",1);

    save_drs_section_btn = uicontrol("Parent", fig, "Style", "pushbutton", "String", "Save DRS Section", ...
        'Units','normalized', 'Position',[0.8 0.12 0.1 0.06]);

    save_file_btn = uicontrol("Parent", fig, "Style", "pushbutton", "String", "Save File", ...
        'Units','normalized', 'Position',[0.8 0.05 0.1 0.06]);
    
    % Update the plot based on slider values
    addlistener(start_point, 'Value', 'PostSet', @(src, event) updatePlot());
    addlistener(end_point, 'Value', 'PostSet', @(src, event) updatePlot());
    save_drs_section_btn.Callback = @save_drs_section;
    save_file_btn.Callback = @save_file;
    
    
    function updatePlot()
        start_idx = round(start_point.Value);
        end_idx  = round(end_point.Value);
        % Update the displayed section of the track
        drs_section.XData = TrackInfo.coords(start_idx:end_idx, 1);
        drs_section.YData = TrackInfo.coords(start_idx:end_idx, 2);
    end

function save_drs_section(~, ~)
        start_idx = round(start_point.Value);
        end_idx  = round(end_point.Value);
        TrackInfo.drs(start_idx:end_idx) = true;
        plot(ax, TrackInfo.coords(start_idx:end_idx, 1), TrackInfo.coords(start_idx:end_idx, 2), "y")
    end

    function save_file(~, ~)
        save(track + '_drs', 'TrackInfo');
    end
end

main("bluemax")