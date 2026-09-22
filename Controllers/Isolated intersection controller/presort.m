% PRE-SORTING LOGIC FOR CAVs
% This script aligns vehicles to the correct lane for their route 
% based on the intersection's controlled links.

% 1. Get the current list of all vehicles
vehIDs = traci.vehicle.getIDList();

% 2. Get the connectivity mapping of the intersection
% Each link is {incoming_lane, outgoing_lane, junction_internal_lane}

for i = 1:length(vehIDs)
    veh = vehIDs{i};
    nextTLS = traci.vehicle.getNextTLS(veh);

    if ~isempty(nextTLS)
    nextTLS = nextTLS{1};
    nextTLS = nextTLS{1};

    
    % Get vehicle route info
    currentRoad = traci.vehicle.getRoadID(veh);
    route = traci.vehicle.getRoute(veh);
    
    % Determine the 'next edge' in the route
    [~, idx] = ismember(currentRoad, route);
    
    if idx > 0 && idx < length(route)
        nextRoad = route{idx + 1};
        
        % Identify lanes on the current road that connect to the nextRoad
        validLanes = [];
        links = traci.trafficlights.getControlledLinks(nextTLS);
        for j = 1:length(links)
            inL = links{j}{1}; % Incoming (e.g., 'NC_0')
            outL = links{j}{2}; % Outgoing (e.g., 'CE_0')
            
            % Check if this link matches the vehicle's movement
            if startsWith(inL, [currentRoad '_']) && startsWith(outL, [nextRoad '_'])
                % Extract the lane index (the number after the underscore)
                laneParts = strsplit(inL, '_');
                validLanes = [validLanes, str2double(laneParts{end})];
            end
        end
        
        % Remove duplicates from validLanes
        validLanes = unique(validLanes);
        
        % 3. Execute Lane Change if needed
        if ~isempty(validLanes)
            currentLaneID = traci.vehicle.getLaneID(veh);
            laneIDParts = strsplit(currentLaneID, '_');
            currentIdx = str2double(laneIDParts{end});
            
            % If not in a valid lane, move to the nearest valid one
            if ~ismember(currentIdx, validLanes)
                [~, bestMatchIdx] = min(abs(validLanes - currentIdx));
                targetIdx = validLanes(bestMatchIdx);
                
                % Override SUMO's passive lane changing (Mode 512 ignores safety)
                traci.vehicle.setLaneChangeMode(veh, 512);
                traci.vehicle.changeLane(veh, targetIdx, 100);
            end
        end
    end
    end
end
