(define (domain healthcare-problem2_general)
    (:requirements :strips :typing :adl :fluents :action-costs)
    (:types
        ; location we use to indicate position of robots/medical units
        ; patient: atients that need to be accompanied (by robots) to a medical unit
        ; robot: generic robot that can be either to accompany people or move boxes
        ; unit: medical units to which robots bring boxes(with items)/ patients
        ; item: items in the box/unit to deliver to the unit
        ; container: addition from problem1 to allow robot_carriers to move more boxes at once
        patient robot unit box item location container - object

        ; sub class of robots since we want to expand the domain to more kinds of robots
        ; robot_carrier: robot that carries boxes to units
        ; robot_accompany: robot that accompanies patients to units
        robot_carrier robot_accompany - robot

        ; sub class of items since we want to focus on units having a general item and not a specific instance
        ; eg: care about scalpel, no scalpel5
        defibrillator oxygen ventilator feeding_pump IV_pump ECG_machine scalpel forcep - item 
    )

    (:predicates
        ; could be done for each object (eg: at ?patient - patient ?loc - location, at ?robot - robot ?loc - location)
        ; which could be easier to understand but would be pointless as this one line is just as equivalent and more generalizable
        ; (if i add a new object in the types i don't need to also add a new predicate)
        ; is object ?object of type object (robot, patient, unit, box, items) at location ?loc of type location?
        ; used to see if something is at the desired location (eg unit is at specific location)
        (at ?o - object ?loc - location)

        ; are location ?loc1 of type location and location ?loc2 of type location connected? 
        ; used to see if two locations are connected, needed as robots can only move between connected locations(rooms)
        (are_connected ?from - location ?to - location)

        ; is item ?i of type item inside the box ?b of type box?
        ; used to see if an item is in a specific box or not as we need different supplies at different units and we want to bring the right box
        (inside ?i - item ?b - box)

        ; is box ?b of type box empty?
        ; used to see if we can insert new items or if it already has something in it
        (is_empty ?b - box)

        ; does unit ?u of type unit need the item ?i of type item?
        ; used to see wheter the medical unit needs a necessary item
        (need_item ?u - unit ?i - item)

        ; does unit ?u of type unit have the item ?i of type item?
        ; used to keep track of all the items each unit has, needed since we can have more than 1 unit at the same location
        ; so we cannot track the location of the item/box
        (has_item ?u - unit ?i - item)

        ; is the robot that carries box ?b of type robot_carrier free?
        ; used to see if we can pickup a box with the robot or if it already has one
        (carrier_free ?r - robot_carrier)

        ; is the box ?b of type box free?
        ; used to see if the box is ready to be picked up from a robot or if it was already picked up
        ; this is probably useless if we have only 1 robot carrier as we can just check with the on predicate
        ; but is good in case we have multiple robots
        (box_free ?b - box)

        ; is the patient ?p of type patient not being escorted?
        ; used to see if a patient is free or if a robot has already taken charge of him
        (patient_free ?p - patient)

        ; is the escorter robot ?r free?
        ; same as box_free and carrier_free but for the robot_accompany
        (escorter_free ?r - robot_accompany)

        ; is the robot ?r of type robot_accompany escorting patient ?p ?
        ; used to see if the robot is escorting that patient (probably useless for now, but useful if we have more robots
        ; and we need to check if that specific robot is escorting that specific patient)
        (escorting ?r - robot_accompany ?p - patient)

        ; does patient ?p need to reach unit ?u ?
        ; used to see what unit the patient needs to be escorted to
        (needs_to_reach ?p - patient ?u - unit)

        ; has the patient ?p reached the needed unit ?u ?
        ; used to see if the patient has gotten to the unit he wanted
        (has_reached ?p - patient ?u - unit)

        ; does the patient ?p need escorting?
        ; used to see if the patient needs escorting or if it's in the desired unit
        ; probably redundant to has_reached, will see which one works best
        (needs_escorting ?p - patient)

        ; is the container free?
        ; used to see if the container is already being moved by a robot or if it's free
        (container_free ?c - container)

        ; edit of the previous on function, now we don't need to know if the box is on the carrier
        ; but we need to know if the box is on the container
        (on ?b - box ?c - container)

        ; is the robot carrying the container?
        ; modified version of the on predicate of problem1
        (carrying_container ?r - robot_carrier ?c - container)
    )


    (:functions
        ; what is the container's capacity? (max weight)
        ; used to see if the container reached its limit of weight
        ; might be redundant and using the load could be enough, will see
        (container_capacity ?c - container)

        ; how much is the current container load?
        ; used to check if we can insert another box without going over the limit
        (container_load ?c - container)

        ; since we use metric-ff we need an actual metric to optimize, use total actions
        (total_cost)
    )


    ; function not modified wrt to problem 1 as we still insert items in the boxes
    ; insert an item into a box assuming it's empty and free (might remove the empty if i want to have boxes be able to carry more items)
    (:action insert_item
        :parameters (?r - robot_carrier ?b - box ?i - item ?loc - location)
        :precondition (and
            ; check that the box is free and empty
            (is_empty ?b) (box_free ?b) (not (inside ?i ?b))
            ; maybe add 
            ; (carrier_free ?r)

            ; make sure the item is not in any other box
            (forall (?localb - box) (not (inside ?i ?localb)) )

            ; check that the robot, box and item are at the same location
            (at ?r ?loc) (at ?b ?loc) (at ?i ?loc)
        )
        :effect (and 
            ; make the box not empty and make the item inside the box
            (not (is_empty ?b)) (inside ?i ?b)

            (increase (total_cost) 1)
        )
    )

    ; insert a box into a container
    (:action insert_box
        :parameters (?r - robot_carrier ?b - box ?c - container ?loc - location)
        :precondition (and 
            ; check that the box is free and not empty
            ; similar to later, if the box is empty it will be left at the location which might be an issue, will see
            ;(not (is_empty ?b)) (box_free ?b)
            ; check that the box is not free (picked up by the robot), can probably remove the not free

            ; check that the container is free
            (container_free ?c) (box_free ?b) (carrier_free ?r)

            ; check that the box is not already on the container (can probably remove since carrying_box already implies it)
            ;(not (on ?b ?c))
            ;(forall (?localb - box) (not (on ?localb ?c)) )

            ; check that adding a(1) box to the current load of the container doesn't overflow
            ; note: using >= as we want to reach the full capacity(container is full)
            ; in this case we also assume the capacity to be in boxes number and not in weight (hence increase by 1)
            (>= (container_capacity ?c) (+ (container_load ?c) 1))

            ; check that the robot, box and container are at the same location
            (at ?r ?loc) (at ?b ?loc) (at ?c ?loc)
        )
        :effect (and 
            ; make the box to be onto the container
            (on ?b ?c)

            ; increase the current load of the container by 1 box
            ; could either increase the load or decrease the capacity
            ; should be equivalent, personally prefer to reason on reaching capacity rather than reaching zero
            (increase (container_load ?c) 1)

            ; box stays not free as it was loaded onto the container
            (not (box_free ?b))

            (increase (total_cost) 1)
        )
    )


    ; make a free robot pick up a free container at a location
    ; modified pickup function of the problem1
    (:action pickup_container
        :parameters (?r - robot_carrier ?c - container ?loc - location)
        :precondition (and 
            ; check that the carrier and the container are free
            (carrier_free ?r) (container_free ?c)

            ; check that the container is not empty
            ; probably creates issues as if it delivers a box and the container becomes empty
            ; the robot will not pick it back up and leave it at the unit (wrong since we only have 1 container per robot)
            ; might make it a dual function with pickup_empty and pickup to make sure there are no issues, will see if this works already or not first
            ;(not (container_empty ?c))
            
            ; check that the robot and box are at the same location
            (at ?r ?loc) (at ?c ?loc)
        )
        :effect (and 
            ; make the robot and the container not free and set robot to be carrying the container
            (not (carrier_free ?r)) (not (container_free ?c)) (carrying_container ?r ?c)

            (increase (total_cost) 1)
        )
    )
    

    ; make the robot carrying a continer move from one location to another, also update the position of the container being carried and box in container
    ; container and box update is needed as in the delivery function i check that the box is also at the correct location
    (:action move_busy_carrier
        :parameters (?r - robot_carrier ?from - location ?to - location ?b - box ?c - container)
        :precondition (and
            ; check that the two locations are connected
            (are_connected ?from ?to)

            ; check that the box is on the container
            ; check that the robot is carrying the container
            (on ?b ?c) (carrying_container ?r ?c) ;(not (carrier_free ?r)) (not (container_free ?c))

            ; check that the robot, container and box are at starting location
            (at ?r ?from) (at ?b ?from) (at ?c ?from)
        )
        :effect (and 
            ; make the robot not be at starting location and make it be at ending location
            (not (at ?r ?from)) (at ?r ?to)

            ; make the box not be at starting location and make it be at ending location
            (not (at ?b ?from)) (at ?b ?to)
            
            ; make the container not be at starting location and make it be at ending location
            (not (at ?c ?from)) (at ?c ?to)

            (increase (total_cost) 1)
        )
    )


    ; opposite of pickup
    ; make a busy robot deploy a container at a location
    (:action deploy_container
        :parameters (?r - robot_carrier ?c - container ?loc - location)
        :precondition (and 
            ; check that the robot and the container are not free(busy) and that the robot is carrying the container
            ;(not (carrier_free ?r)) (not (container_free ?c)) 
            (carrying_container ?r ?c)

            ; check that the robot and container are at the same location
            (at ?r ?loc) (at ?c ?loc)
        )
        :effect (and 
            ; make the robot and contaier free and set the robot to not be carrying the container
            (carrier_free ?r) (not (carrying_container ?r ?c)) (container_free ?c)

            (increase (total_cost) 1)
        )
    )


    (:action deploy_box
        :parameters (?r - robot_carrier ?c - container ?b - box ?loc - location)
        :precondition (and 
            ; check that the carrier and container are free and that the box is still busy and on the container
            (carrier_free ?r) (container_free ?c) (not (box_free ?b)) (on ?b ?c)

            ; check that robot, container and box are at the same location
            (at ?r ?loc) (at ?c ?loc) (at ?b ?loc)
        )
        :effect (and 
            (box_free ?b) (not (on ?b ?c)) (carrier_free ?r)

            (decrease (container_load ?c) 1)

            (increase (total_cost) 1)
        )
    )
    

    ; delivers an item from a box to a unit that needs it and does not have it
    (:action deliver_item
        :parameters (?r - robot_carrier ?b - box ?loc - location ?i - item ?u - unit); ?c - container)
        :precondition (and 
            ; check if unit needs the item and that the unit does not have the item (probably redundant, might delete later)
            (need_item ?u ?i) (not (has_item ?u ?i))

            ; check if the item is inside the box, could and (not (is_empty ?b)) to make sure, but would probably be redundant
            (inside ?i ?b)

            ; check to make sure the box is not on the robot, as it cannot deliver an item if it's holding the box
            ;(not (on ?b ?c))

            ; check that the robot and the carrier are free
            (box_free ?b) (carrier_free ?r)

            ; check that the robot, box and unit are at the same location
            (at ?r ?loc) (at ?b ?loc) (at ?u ?loc)
        )
        :effect (and
            ; make the unit not need the item and indicates that is now has the item
            (not (need_item ?u ?i)) (has_item ?u ?i)

            ; make the item not be inside the box anymore as we just removed it to deliver it to the unit(meaning the box is now empty)
            (not (inside ?i ?b)) (is_empty ?b)

            (increase (total_cost) 1)
        )
    )
    

    ; same as move_busy_carrier but with the robot not holding a box
    (:action move_free_carrier
        :parameters (?r - robot_carrier ?from - location ?to - location)
        :precondition (and 
            ; check that the two locations are connected
            (are_connected ?from ?to)

            ; since we have a function that moves the robot holding a box, we make sure for this one the robot does not hold a box
            (carrier_free ?r)

            ; check that the robot is at starting location
            (at ?r ?from)
        )
        :effect (and 
            ; make the robot not be at starting location and make it be at ending location
            (not (at ?r ?from)) (at ?r ?to)

            (increase (total_cost) 1)
        )
    )


    ; ====================
    ; moved all patient-related functions at the end since the task is only box-related and there is no need to modify the following functions


    ; make a free robot move from one location to another (to go fetch a patient)
    ; honestly not sure if i need this function or if i can just create a move_free_robot and pass the general robot class
    ; instead of having one per accompany and one per carrier as they are identical except for the subclass
    (:action move_free_accompanier
        :parameters (?r - robot_accompany ?from - location ?to - location)
        :precondition (and 
            ; check that the robot is at starting location
            (at ?r ?from)

            ; check that the two locations are connected
            (are_connected ?from ?to)

            ; since we have a function that moves the robot holding a box, we make sure for this one the robot does not hold a box
            (escorter_free ?r)
        )
        :effect (and 
            ; make the robot not be at starting location and make it be at ending location
            (not (at ?r ?from)) (at ?r ?to)

            (increase (total_cost) 1)
        )
    )


    ; make a robot that is escorting a person move from one location to another
    (:action move_escorting_accompanier
        :parameters (?r - robot_accompany ?from - location ?to - location ?p - patient)
        :precondition (and 
            ; check that the robot is escorting a patient
            (escorting ?r ?p) (not (escorter_free ?r)) (not (patient_free ?p))

            ; check that the two locations are connected
            (are_connected ?from ?to)

            ; check that robot and patient are at starting location
            (at ?r ?from) (at ?p ?from)
        )
        :effect (and 
            ; make the robot not be at starting location and make it be at ending location
            (not (at ?r ?from)) (at ?r ?to)

            ; make the patient not be at starting location and make it be at ending location
            (not (at ?p ?from)) (at ?p ?to)

            (increase (total_cost) 1)
        )
    )


    ; TODO 
    ; make a free robot get a patient at a location
    (:action get_patient
        :parameters (?r - robot_accompany ?p - patient ?loc - location)
        :precondition (and 
            ; check that both the patient and the robot are free (not being escorted and not escorting anyone)
            (patient_free ?p) (escorter_free ?r) (needs_escorting ?p)

            ; check that the robot and the patient are at same location
            (at ?r ?loc) (at ?p ?loc)
        )
        :effect (and 
            ; make the patient being escorted and the robot to be busy
            (not (patient_free ?p)) (not (escorter_free ?r)) (escorting ?r ?p)

            (increase (total_cost) 1)
        )
    )


    ; TODO this drops off a patient only if it's in the desired(needs_to_reach) unit, if test fails separate this to make it able to drop off
    ; patients in any location/unit and see if it makes a difference
    ; make a robot drop a patient off at the desired unit in the location (need unit since there are many units at same location)
    (:action drop_off_patient
        :parameters (?r - robot_accompany ?p - patient ?loc - location ?u - unit)
        :precondition (and 
            ; check that the robot is escorting the patient and that the patient actually needed to get to this unit
            (escorting ?r ?p) (not (escorter_free ?r)) (not (patient_free ?p)) (needs_to_reach ?p ?u)

            ; check that the robot, patient and unit are at the same location
            (at ?r ?loc) (at ?p ?loc) (at ?u ?loc)
        )
        :effect (and 
            (not (escorting ?r ?p)) (escorter_free ?r) (has_reached ?p ?u) (patient_free ?p) (not (needs_escorting ?p)) ;not( (needs_to_reach ?p ?u))

            (increase (total_cost) 1)
        )
    )


)