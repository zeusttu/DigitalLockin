classdef PXILockIn < qd.classes.ComInstrument
    %PXILOCKIN Digital Lock-In using NI-PXI chassis
    %   This is a software-based virtual Lock-in amplifier
    %   It uses a National Instruments PXI chassis with waveform generators
    %   and signal analysers to generate and measurem signals. All the
    %   processing is done in software on the computer, by a virtual
    %   instrument (called DigitalLockin) written in Python.
    %   
    %   Note that an instance of this instrument encompasses a full
    %   instance of the virtual lock-in software which governs a full PXI
    %   chassis and may contain several virtual lock-in amplifiers. You can
    %   select one by writing to the 'SELECT' channel.
    %     Supported commands:
    % 	SELECT
    % 		Select a lock-in instrument
    % 	SET F|FS|A|T|PHASEOFFSET|MEASCH <value>
    % 		Set the value of a variable of the selected lock-in instrument
    % 	GET RPHIBUFFER
    % 		Get multiline representation of all values of R and PHI acquired from
    % 		the selected lock-in instrument since last time they were queried
    % 	GET RPHI|R|PHI|XY|X|Y
    % 		Get the last acquired values of R/PHI/X/Y from the selected lock-in
    % 		This function may still return multiple values (comma-separated) when
    % 		multiple channels are in use
    % 	GET F|FS|A|T|PHASEOFFSET
    % 		Get value of excitation/measurement control variable of selected lock-in instrument
    % 	START
    % 		Start measurements on the selected lock-in instrument
    % 	STOP
    % 		Stop measurements on the selected lock-in instrument
    % 	CLOSE
    % 		Close the selected lock-in instrument
    % 	CLOSE ALL
    % 		Close all lock-in instruments and exit
    % 	PHASENULL
    % 		Set the phase offset of the current lock-in instrument to the current phase
    %   
    %   TODO:
    %     * Create separate channels for each lock-in instead of requiring
    %       a manual setc('SELECT', <li>). E.g. one could make '1R', '2R',
    %       '1PHI', '2PHI' channels to specify which lock-in to take a
    %       channel from. [DONE for GET, not needed for SET]
    %     * Cache PHI when only R is requested and cache R when only PHI is
    %       requested. Also do some bookkeeping on which has already been
    %       requested and which hasn't
    %     * Make separate channels for each physical channel, i.e. instead
    %       of returning R and PHI as arrays make each element individually
    %       accessible as an instrument channel. One could make 'R1', 'R2',
    %       'PHI1', 'PHI2' etc., or when combined with the first suggestion
    %       there would be '1R1', '2PHI1', '1PHI2' etc. Of course, any
    %       channel that has not been requested yet should be cached, and
    %       bookkeeping should be done of what has and has not been
    %       requested.
    %     * Maybe implement GET RPHIBUFFER
    %       Though it seems unlikely that this will ever be used
    
    properties (Constant, GetAccess=private)
        GENERIC_ERROR = 'An error occured. Check the virtual instrument window for details.';
    end
    
    properties (GetAccess=private)
        r = [];
        phi = [];
        rphixyread = [];
        numsetsubchans = 1;
    end
    
    methods
        function obj = PXILockIn(com)
        % Constructor
            pcom = serial(com);
            obj@qd.classes.ComInstrument(pcom);
        end
        
        function rep = query(obj, req)
        % Send command and read the response.
        % Overridden because %s format string removed whitespace in the
        % middle of the response.
        % This function may be removed if the right behaviour is
        % implemented in ComInstrument.
        % This function also implements a debugging message but that does
        % not have to go into the ComInstrument implementation
            rep = query(obj.com, req, '%s\n', '%c');
            if rep(end) == 10
                rep = rep(1:end-1);
            end
            %fprintf(1, '%s     :     %s\n', req, rep);
        end
        
        function r = model(obj)
        % Query the instrument for its make and model
            r = obj.query('*IDN?');
        end
        
        function r = default_name(obj)
        % Default name the instrument gets when no name is specified by the
        % user
            r = 'DigitalLockin';
        end

        function r = channels(obj)
        % Get list of available channels. Needed because getc and setc will
        % fail if the requested channel is not in this list, even if it
        % exists and is readable/writable.
            r = {'rphibuffer' 'rphi' 'r' 'phi' 'xy' 'x' 'y' 'f' 'fs' 'a' 't' 'phaseoffset' 'measch' 'select'};
            if isempty(obj.rphixyread)
                lastsubchan = 2*obj.numsetsubchans+1;
            else
                lastsubchan = (size(obj.rphixyread,2) - 1) / 2;
            end
            for cnt = 0:lastsubchan
                r{end+1} = ['r(' num2str(cnt) ')'];
                r{end+1} = ['phi(' num2str(cnt) ')'];
                r{end+1} = ['x(' num2str(cnt) ')'];
                r{end+1} = ['y(' num2str(cnt) ')'];
            end
        end

        function val = getc(obj, channel)
        % Get the value of a channel
        % If the channel name is prefixed with a number, get the channel
        % from the lock-in corresponding with that number
        % Note that this selects the corresponding lock-in, and does not
        % return to the previously selected one afterward
            firstnonnumchar = 1;
            while ~isnan(str2double(channel(1:firstnonnumchar)))
                firstnonnumchar = firstnonnumchar + 1;
            end
            if firstnonnumchar > 1;
                obj.set('SELECT', str2double(channel(1:firstnonnumchar-1)));
                channel = channel(firstnonnumchar:end);
            end
            switch upper(channel)
                case 'RPHIBUFFER'
                    error('DigitalLockin.GETC(RPHIBUFFER): Not yet implemented');
                case {'RPHI', 'XY'}
                    response = obj.query(['GET ' channel]);
                    if length(response) > 2 && strcmp(response(1:2), 'OK')
                        val = str2num(response(4:end));
                        obj.rphixyread = ones(4, length(val));
                    else
                        obj.rphixyread = [];
                        error(['DigitalLockin.GETC(%s): ' obj.GENERIC_ERROR]);
                    end
                case {'R', 'PHI', 'X', 'Y'}
                    %warning('DigitalLockin.GETC(%s): Requesting just one output channel at a time is not recommended unless you do not wish to use any other variables', channel);
                    response = obj.query(['GET ' channel]);
                    if length(response) > 2 && strcmp(response(1:2), 'OK')
                        val = str2num(response(4:end));
                    else
                        error(['DigitalLockin.GETC(%s): ' obj.GENERIC_ERROR], channel);
                    end
                case {'F', 'FS', 'A', 'T', 'PHASEOFFSET'}
                    try
                        val = obj.querym(['GET ' channel], 'OK %f');
                    catch err
                        error('DigitalLockin.GETC(%s): %s Additional info from Matlab: %s', channel, obj.GENERIC_ERROR, err.message);
                    end
                otherwise
                    % TODO reset stuff on getc(rphi)
                    if ~isempty(regexpi(channel, '(r|phi|x|y)\([0-9]+\)'))
                        ch = str2double(channel(end-1)); % Assume at most 10 physical channels
                        switch upper(channel(1))
                            case 'R'
                                readboolidx = 1;
                            case 'P'
                                readboolidx = 2;
                            case 'X'
                                readboolidx = 3;
                            otherwise
                                readboolidx = 4;
                        end
                        if isempty(obj.rphixyread) || obj.rphixyread(readboolidx, ch+1)
                            rphi = obj.getc('rphi');
                            if isempty(obj.r)
                                obj.r = rphi(1:(end+1)/2);
                            else
                                alpha = 1;
                                obj.r = (1-alpha) * obj.r + alpha * rphi(1:(end+1)/2);
                            end
                            obj.phi = rphi((end+3)/2:end);
                            obj.rphixyread(:,:) = 0;
                        end
                        obj.rphixyread(readboolidx, ch+1) = 1;
                        switch upper(channel(1))
                            case 'R'
                                val = obj.r(ch+1);
                            case 'P'
                                val = obj.phi(ch);
                            case 'X'
                                val = obj.r(ch+1) * cos(obj.phi(ch));
                            otherwise
                                val = obj.r(ch+1) * sin(obj.phi(ch));
                        end
                    else
                        error('DigitalLockin.GETC(%s): Unsupported channel.', channel);
                    end
            end
            %fprintf(1, 'GET(%s) = %s\n', channel, num2str(val));
        end

        function setc(obj, channel, val)
            switch upper(channel)
                case 'SELECT'
                    comcmd = ['SELECT ' num2str(val)];
                case {'F', 'FS', 'A', 'T'}
                    comcmd = ['SET ' channel ' ' num2str(val)];
                case 'MEASCH'
                    comcmd = num2str(val, '%d,');
                    if comcmd(end) == ','
                        comcmd = comcmd(1:end-1); % Remove comma
                    end
                    obj.numsetsubchans = length(strsplit(comcmd, ','));
                    comcmd = ['SET MEASCH ' comcmd];
                case 'PHASEOFFSET'
                    if strcmp(num2str(val), 'now')
                        comcmd = 'PHASENULL';
                    else
                        comcmd = ['SET ' channel ' ' num2str(val)];
                    end
                otherwise
                    error('DigitalLockin.SETC(%s=%s): Unsupported channel.', channel, val);
            end
            if strcmp(obj.query(comcmd), 'ERROR')
                error('DigitalLockin.SETC(%s=%s): %s.', channel, num2str(val), obj.GENERIC_ERROR);
            end
        end
        
        function start_lockin(obj)
            if strcmp(obj.query('START'), 'ERROR')
                error('DigitalLockin.SELECT(%s): %s.', num2str(value), obj.GENERIC_ERROR);
            end
        end
        
        function stop_lockin(obj)
            if strcmp(obj.query('STOP'), 'ERROR')
                error('DigitalLockin.STOP_LOCKIN(): %s.', obj.GENERIC_ERROR);
            end
        end
        
        function close_all(obj)
            if strcmp(obj.query('CLOSE ALL'), 'ERROR')
                error('DigitalLockin.CLOSE_ALL(): %s.', num2str(value), obj.GENERIC_ERROR);
            end
            obj.read();
            obj.delete();
        end
    end
    
end

