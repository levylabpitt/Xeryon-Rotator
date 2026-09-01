%% SETUP
% Xeryon.py needs to be in this folder.
% "pyserial" library is required. (= NOT serial library)
library = py.importlib.import_module('Xeryon');

controller = library.Xeryon("COM1", 115200);       % Setup serial communication (COM-port, baudrate)
% The line below changed a bit from previous versions, you now need to specify the axis letter.
% You can find the axis name in the config.txt file that is provided with the stages.
axisX = controller.addAxis(library.Stage.XLS_312, "X");  % Add correct stage
axisY = controller.addAxis(library.Stage.XRTU_40_3, "Y");


% TIP: type: "library.Stage." and press TAB, this gives a list of all
% possible stages. You can also use this with "axisX." to see which
% functions you can use.
axisX.setUnits(library.Units.mm)                    % Initialize units
%% First movements
controller.start() % This starts up all the communication and reads the settings
try
%%
   % axisX.findIndex()  % This finds the index for this axis.
                       % The stage now starts moving.
%% Basic control
    % This includes the functions: setDPOS(..), getDPOS(), getEPOS(), step(..),
    % setUnits(..)
    % DPOS stands for 'Desired POSition', this is the position that the stage trys to reach.
    % EPOS stands for 'Encoder POSition', this value indicates the actual position of the stage.
    axisX.setUnits(library.Units.mm)
    axisX.setDPOS(5)
    axisX.setDPOS(-5)
%%
    axisX.setUnits(library.Units.mu)
    axisX.setDPOS(50)
%%
    disp(axisX.getDPOS())
    disp(axisX.getEPOS())
%%
    axisX.setUnits(library.Units.mm)
    axisX.setDPOS(0)
    for i = 0:5
        axisX.step(1)    %Step 5x 1mm
    end
%% Speed control and scanning
    % Scanning: continously move with fixed speed
    axisX.setDPOS(0)
    axisX.setUnits(library.Units.mm)
    axisX.setSpeed(1)      % The speed value is set in the current unit. So here it is 1 mm/s.
    axisX.startScan(-1)    % startScan() takes a negative or positive value.
%%
    axisX.stopScan()
    axisX.setDPOS(0)      % Notice that the speed is still 1 mm/s.
%%
    axisX.startScan(1, 2) % It is also possible to scan for a certain amount of seconds.
%% Getting data back from the controller
    % Different status bits (defined in the datasheet):
    axisX.isForceZero()
    axisX.isMotorOn()
    axisX.isClosedLoop()
    axisX.isEncoderAtIndex()
    axisX.isEncoderValid()
    axisX.isSearchingIndex()
    axisX.isPositionReached()
    axisX.isEncoderError()
    axisX.isScanning()
    axisX.isAtLeftEnd()
    axisX.isAtRightEnd()
    axisX.isErrorLimit()
    axisX.isSearchingOptimalFrequency()
%% Logging and plotting data
    % The function 'endLogging()' returns a dictionary of the format:
    % {"EPOS": [..,..,..], "DPOS": [..,..,..], "STAT": [..,..,..]}
    axisX.setUnits(library.Units.mm)
    axisX.setSpeed(50)
    axisX.setDPOS(0)
    axisX.setSpeed(120)
    axisX.startLogging()
    axisX.step(0.01)
    logs = axisX.endLogging();
    axisX.setSpeed(50)
    axisX.setDPOS(0)
%% Make a plot
    log_epos = struct(logs).EPOS; % Returns python list
    conv_epos = [];
    % convert units:

    for i=1:int64(py.len(log_epos))
        conv_epos(i) = axisX.convertEncoderUnitsToUnits(log_epos{i}, axisX.getUnit())
    end
    plot(conv_epos)
    ylabel(strcat("EPOS (", string(py.str(axisX.getUnit())), ")"))
    xlabel("Sample")
    title("Encoder position")
%% Stop control
    controller.stop()
catch ME
    warning("Something went wrong...")
    controller.stop()
    rethrow(ME)
end