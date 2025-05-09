% input:
%       experimentPbFile: the name of the experiment without extension (char)
%       ExperimentPexFile: the name of the experiment with the extension '.pex' 
%       roadPexFile: the path to the template pex file like:  ['C:\Users\adeye\Desktop\real_world_data\TemplatePexFile\TemplatePexFile.pex']
%       options: struct : options of the road with:
%                       options{i}.relativeHeading   [degree]
%                       options{i}.deltaX      [meter]
%                       options{i}.deltaY      [meter]
%                       options{i}.deltaZ      [meter]
%                       options{i}.EntryTension     [meter]
%                       options{i}.ExitTension        [meter]

function writeBezierRoadToPexFile(ExperimentPBFile,ExperimentPexFile,RoadPexFile,options)
  
%load all files
pexFileName=ExperimentPexFile;
pbFileName=ExperimentPBFile;
RoadTemplate=RoadPexFile;
    
%Make backup of the current PEX file
backupFolderPath = [pwd '\BackupPEXFiles'];
if ~exist(backupFolderPath)
    mkdir(backupFolderPath);
end

currentTime = datestr(now,'yy-mm-dd_HH-MM-SS');
copyfile([pwd '\' pexFileName], [backupFolderPath '\Backup_' currentTime '_' pexFileName]);


%Convert PEX to STRUCT
disp('Loading the experiment and template PEX files...') %message for the commande
loadedPexFile = xml2struct(pexFileName); 
loadedTemplate = xml2struct(RoadTemplate);

%get all objects on the pb file
myExp = prescan.experiment.readDataModels(pbFileName);
allExpObjects = myExp.worldmodel.object; %On this varaible we have all objects on the pb file (roads, trees, cars ...)
roadIndex = 1; %counter for number of roads
nbBezier=1; %counter for number of Bezier road
indexroadsadd=1;

for i=1:length(allExpObjects)
    
    %we would like modify only road add on pb file and count the number of
    %road and the number of Bezier road on Pex file
    objectTypeName=allExpObjects{i,1}.objectTypeName;
    if  not(strcmp(objectTypeName, 'Road' ))
        if strcmp(objectTypeName,'BezierRoad')
            roadIndex= roadIndex+1;
            nbBezier=nbBezier+1;
         elseif strcmp(objectTypeName,'XCrossing')
            roadIndex= roadIndex+1;
        elseif strcmp(objectTypeName,'YCrossing')
            roadIndex= roadIndex+1;
         elseif strcmp(objectTypeName,'Roundabout')
            roadIndex= roadIndex+1;
         elseif strcmp(objectTypeName,'CubicSplineRoad')
            roadIndex= roadIndex+1;
         elseif strcmp(objectTypeName,'StraightRoad')
            roadIndex=roadIndex+1;
        end
    else
    
    %Get Road properties from PB file
    currentObjectUniqueID = allExpObjects{i,1}.uniqueID;
    currentObjectNumericalID= allExpObjects{i,1}.numericalID;
    currentOjectTypeId= allExpObjects{i,1}.objectTypeID;
    currentObjectPosition = allExpObjects{i,1}.pose.position;
    currentObjectOrientation = allExpObjects{i,1}.pose.orientation;
    
    %Get the correct road template  
    currentRoadStruct=getCorrectRoadStruct('BezierRoad',loadedTemplate);
    
        %Set the correct properties for Bezier road in the STRUCT
        currentRoadStruct.Attributes.id = strcat('CurvedRoad_',num2str(nbBezier));
        currentRoadStruct.Attributes.NumericalID = num2str(currentObjectNumericalID);
        currentRoadStruct.Attributes.UniqueId = num2str(currentObjectUniqueID);
        currentRoadStruct.Attributes.ObjectTypeID=num2str(currentOjectTypeId);
        currentRoadStruct.Attributes.RelativeHeading=num2str(options{indexroadsadd}.relativeHeading);
        currentRoadStruct.Attributes.Xoffset=num2str(options{indexroadsadd}.deltaX);
        currentRoadStruct.Attributes.Yoffset=num2str(options{indexroadsadd}.deltaY);
        currentRoadStruct.Attributes.Zoffset=num2str(options{indexroadsadd}.deltaZ);
        currentRoadStruct.Attributes.ControlPoint1Distance=num2str(options{indexroadsadd}.EntryTension);
        currentRoadStruct.Attributes.ControlPoint2Distance=num2str(options{indexroadsadd}.ExitTension);
        
        currentRoadStruct.Location.Attributes.X = num2str(currentObjectPosition.x);
        currentRoadStruct.Location.Attributes.Y = num2str(currentObjectPosition.y);
        currentRoadStruct.Location.Attributes.Z = num2str(currentObjectPosition.z);
        
        %currentRoadStruct.Orientation.Attributes.Bank = num2str(rad2deg(currentObjectOrientation.roll));
        currentRoadStruct.Orientation.Attributes.Heading = num2str(currentObjectOrientation.yaw);
        %currentRoadStruct.Orientation.Attributes.Tilt = num2str(rad2deg(currentObjectOrientation.pitch));
        
        
        currentRoadStruct.CentralLineDefinition.Attributes.UniqueId=num2str(500*roadIndex);
        currentRoadStruct.LaneLineDefinitions.LaneLineDefinition.Attributes.UniqueId=num2str(500*roadIndex+1);
        currentRoadStruct.CurbLineDefinitions.LaneLineDefinition{1,1}.Attributes.UniqueId=num2str(500*roadIndex+2);
        currentRoadStruct.CurbLineDefinitions.LaneLineDefinition{1,2}.Attributes.UniqueId=num2str(5000*roadIndex+3);
        
        %add properties to the pex file convert into structure
        if i==1 & roadIndex==2 %this test is true when there already is one road on the Pex file
            RoadInformations= loadedPexFile.Experiment.InfraStructure.RoadSegments.RoadSegment; %we save informations of the road
            loadedPexFile.Experiment.InfraStructure.RoadSegments.RoadSegment={}; % We supress informations to have an array
            loadedPexFile.Experiment.InfraStructure.RoadSegments.RoadSegment{1,1}=RoadInformations; %we add on the first position of the array informations of the road
        end
        
        loadedPexFile.Experiment.InfraStructure.RoadSegments.RoadSegment{1,roadIndex} = currentRoadStruct;
        roadIndex = roadIndex + 1;
        indexroadsadd=indexroadsadd+1;
        nbBezier=nbBezier+1;
        
    end   
 end
        


%Convert the populated STRUCT to PEX
disp('Overwriting changes to PEX file...')
struct2xml(loadedPexFile,pexFileName)



%Overwriting the current PEX file with the populated one
copyfile([pwd '\' pexFileName '.xml'], [pwd '\' pexFileName]);

disp(['Done...A back up of original PEX file is made at: ' backupFolderPath '\Backup_' currentTime '_' pexFileName]);
end
