function saveResults( FolderName, fileName,overWrite,soln,OCP,Notes)

varNames = {'soln','OCP','Notes'};
% Check if folder exists if it doesn't make it
if ~exist(FolderName,'dir')
    warning('Warning: directory does not exist:%s\n. It will be created.', FolderName)
    mkdir(FolderName);
end

savedResults = 0;
idxFile = 1;
while ~savedResults
    
    % Build full file name
    fullFileName = fullfile(FolderName,strcat(fileName,num2str(idxFile),'.mat'));
  
    % Check if file index
    if exist(fullFileName, 'file')
        % File exists --> increase index count
        idxFile = idxFile + 1;
    else
        % File does not exist.
        if overWrite
           fullFileName = fullfile(FolderName,strcat(fileName,num2str(idxFile-1),'.mat'));
        end
        save(fullFileName,varNames{:})
        fprintf('File "%s" was succesfully saved!\n',fullFileName);
        savedResults = 1;
    end
end

end

