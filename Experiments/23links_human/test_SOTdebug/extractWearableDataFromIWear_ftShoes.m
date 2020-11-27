

%% Load and read wearable file

bucket.LOGfilename = fullfile(bucket.pathToWearableDataFtShoes, 'data.log');
fileID = fopen(bucket.LOGfilename);
formatSpec = '%s';
tmp.file = textscan(fileID, formatSpec,'MultipleDelimsAsOne', 1, 'Delimiter', {' '});
fclose(fileID);

% Remove and replace symbols
tmp.match = ('((');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('))');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('"');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);

tmp.oldStr = '()';
tmp.newStr = 'empty';
tmp.file{1,1} = replace(tmp.file{1,1}, tmp.oldStr, tmp.newStr);

tmp.match = (')');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);
tmp.match = ('(');
tmp.file{1,1} = erase(tmp.file{1,1},tmp.match);

% Check for the repeated values
for fileIdx = 1 : length(tmp.file{1,1})-1
    if strcmp(tmp.file{1, 1}{fileIdx,1},tmp.file{1, 1}{fileIdx+1,1})
        TF = isstrprop(tmp.file{1, 1}{fileIdx,1},'digit');
        if ~any(TF)
            tmp.file{1, 1}{fileIdx,1} = 'repeatedValue';
        end
    end
end

for fileIdx = 1 : length(tmp.file{1,1})-1
    if strcmp(tmp.file{1, 1}{fileIdx,1},tmp.file{1, 1}{fileIdx+1,1}) ...
            && ~isempty(strfind(tmp.file{1, 1}{fileIdx+1,1},'::'))
        tmp.file{1, 1}{fileIdx,1} = 'repeatedValue';
    end
end

%% Create data struct
if opts.left
    tmp.ftShoeIndx = find(strcmp(tmp.file{1, 1}, 'FTShoeLeft::'));
    wearData.nrOfFramesFromShoe   = length(tmp.ftShoeIndx);
    tmp.leftFtShoeIndx  = find(strcmp(tmp.file{1, 1}, 'FTShoeLeftFTSensors'));
    
    % % % ----Check if values are missing in the left shoe
    % % flag.missingLeftValues = false; % assumed no value is missing as default
    % % if length(tmp.leftFtShoeIndx) ~= wearData.nrOfFrames
    % %     disp('[Info]: Missing values for the Left shoe!');
    % %     [tmp.missingLeftShoeIdx,flag.missingLeftValues] = checkingMissingValues(tmp.file{1, 1}, ...
    % %         tmp.leftFtShoeIndx, wearData.nrOfFrames, 5, 'FTShoeLeftFTSensors');
    % % end
    
    % ----Fill wearData for the shoes
    wearData.ftShoes.Left  = zeros(6,length(tmp.leftFtShoeIndx));
    for l_shoeIdx = 1 : length(tmp.leftFtShoeIndx)
        for vect6Idx = 1 : 6
            wearData.ftShoes.Left(vect6Idx,l_shoeIdx)  = str2num(tmp.file{1, 1}{tmp.leftFtShoeIndx(l_shoeIdx)+(vect6Idx+1),1});
        end
    end
    % % % ----If values missing, fill them with the previous valid one
    % % % Left
    % % if flag.missingLeftValues
    % %     wearData.ftShoes.Left = fillMissingValues(tmp.leftFtShoeIndx,tmp.missingLeftShoeIdx,wearData.ftShoes.Left);
    % % end
else
    tmp.ftShoeIndx = find(strcmp(tmp.file{1, 1}, 'FTShoeRight::'));
    wearData.nrOfFramesFromShoe   = length(tmp.ftShoeIndx);
    tmp.rightFtShoeIndx  = find(strcmp(tmp.file{1, 1}, 'FTShoeRightFTSensors'));

    % % % ----Check if values are missing in the right shoe
    % % flag.missingRightValues = false; % assumed no value is missing as default
    % % if length(tmp.rightFtShoeIndx) ~= wearData.nrOfFrames
    % %     disp('[Info]: Missing values for the Right shoe!');
    % %     [tmp.missingRightShoeIdx,flag.missingRightValues] = checkingMissingValues(tmp.file{1, 1}, ...
    % %         tmp.XsensSuitIndx, wearData.nrOfFrames, 14, 'FTShoeRightFTSensors');
    % % end
    
    % ----Fill wearData for the shoes
    wearData.ftShoes.Right  = zeros(6,length(tmp.rightFtShoeIndx));
    for r_shoeIdx = 1 : length(tmp.rightFtShoeIndx) % Right
        for vect6Idx = 1 : 6
            wearData.ftShoes.Right(vect6Idx,r_shoeIdx)  = str2num(tmp.file{1, 1}{tmp.rightFtShoeIndx(r_shoeIdx)+(vect6Idx+1),1});
        end
    end
    % % % ----If values missing, fill them with the previous valid one
    % % % Right
    % % if flag.missingRightValues
    % %     wearData.ftShoes.Right = fillMissingValues(tmp.rightFtShoeIndx,tmp.missingRightShoeIdx,wearData.ftShoes.Right);
    % % end
end
