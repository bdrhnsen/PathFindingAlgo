classdef PriorityQueue < handle
    properties (Access = private)
        elements
        priorities
        count
    end
    
    methods
        function obj = PriorityQueue()
            obj.elements = [];
            obj.priorities = [];
            obj.count = 0;
        end
        
        function insert(obj, element, priority)
            obj.elements = [obj.elements; element];
            obj.priorities = [obj.priorities; priority];
            obj.count = obj.count + 1;
        end
        
        function element = remove(obj)
            [~, idx] = min(obj.priorities);
            element = obj.elements(idx);
            obj.elements(idx) = [];
            obj.priorities(idx) = [];
            obj.count = obj.count - 1;
        end
        
        function updatePriority(obj, element, newPriority)
            idx = obj.elements == element;
            obj.priorities(idx) = newPriority;
        end
        
        function isEmpty = isEmpty(obj)
            isEmpty = obj.count == 0;
        end
        
        function containsElement = contains(obj, element)
            containsElement = ismember(element, obj.elements);
        end
    end
end
