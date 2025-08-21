(define (domain problem4)
    (:requirements :strips :typing :adl :fluents :durative-actions)
    (:types
        ; location we use to indicate position of robots/medical units
        ; patient: atients that need to be accompanied (by robots) to a medical unit
        ; robot: generic robot that can be either to accompany people or move boxes
        ; unit: medical units to which robots bring boxes(with items)/ patients
        ; item: items in the box/unit to deliver to the unit
        ; container: addition from problem1 to allow robot_carriers to move more boxes at once
        patient robot unit box item location container robot_carrier robot_accompany - object

        ; sub class of robots since we want to expand the domain to more kinds of robots
        ; robot_carrier: robot that carries boxes to units
        ; robot_accompany: robot that accompanies patients to units
        ;robot_carrier robot_accompany - robot

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

        (at_item ?i - item ?loc - location)

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

        ; is the carrier free to do an action?
        ; similar to carrier_free but for the durative actions
        (carrier_free_for_action ?r - robot_carrier)

        ; is the escorter free to do an action?
        ; similar to escorter_free but for the durative action
        (escorter_free_for_action ?r - robot_accompany)
    )


    (:functions
        ; what is the container's capacity? (max weight)
        ; used to see if the container reached its limit of weight
        ; might be redundant and using the load could be enough, will see
        (container_capacity ?c - container)

        ; how much is the current container load?
        ; used to check if we can insert another box without going over the limit
        (container_load ?c - container)
    )



    (:durative-action get_patient
        :parameters (?r - robot_accompany ?p - patient ?loc - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (patient_free ?p))
            (at start (escorter_free ?r))
            (at start (needs_escorting ?p))
            (at start (escorter_free_for_action ?r))
            (at start (at ?r ?loc))
            (at start (at ?p ?loc))

            (over all (at ?r ?loc))
            (over all (at ?p ?loc))

        )
        :effect (and 
            (at start (not (patient_free ?p)))
            (at start (not (escorter_free ?r)))
            (at start (not (escorter_free_for_action ?r)))

            (at end (not (patient_free ?p)))
            (at end (not (escorter_free ?r)))
            (at end (escorting ?r ?p))
            (at end (escorter_free_for_action ?r))
        )
    )

    (:durative-action drop_off_patient
        :parameters (?r - robot_accompany ?p - patient ?loc - location ?u - unit)
        :duration (= ?duration 2)
        :condition (and 
            (at start (escorting ?r ?p))
            (at start (needs_to_reach ?p ?u))
            (at start (escorter_free_for_action ?r))
            (at start (at ?r ?loc))
            (at start (at ?p ?loc))
            (at start (at ?u ?loc))

            (over all (at ?r ?loc))
            (over all (at ?p ?loc))
            (over all (at ?u ?loc))

        )
        :effect (and 
            (at start (not (escorter_free_for_action ?r)))

            (at end (escorter_free_for_action ?r))
            (at end (has_reached ?p ?u))
            (at end (not (needs_escorting ?p)))
            (at end (not (needs_to_reach ?p ?u)))
            (at end (not (escorting ?r ?p)))
            (at end (patient_free ?p))
            (at end (escorter_free ?r))
        )
    )

    (:durative-action move_escorting_accompanier
        :parameters (?r - robot_accompany ?from - location ?to - location ?p - patient)
        :duration (= ?duration 3)
        :condition (and 
            (at start (escorting ?r ?p))
            (at start (escorter_free_for_action ?r))
            (at start (are_connected ?from ?to))
            (at start (at ?r ?from))
            (at start (at ?p ?from))

            (over all (are_connected ?from ?to))
            (over all (escorting ?r ?p))
        )
        :effect (and 
            (at start (not (escorter_free_for_action ?r)))

            (at end (escorter_free_for_action ?r))
            (at end (not (at ?r ?from)))
            (at end (not (at ?p ?from)))

            (at end (at ?r ?to))
            (at end (at ?p ?to))
        )
    )

    (:durative-action move_free_accompanier
        :parameters (?r - robot_accompany ?from - location ?to - location)
        :duration (= ?duration 5)
        :condition (and 
            (at start (escorter_free ?r))
            (at start (escorter_free_for_action ?r))
            (at start (are_connected ?from ?to))
            (at start (at ?r ?from))

            (over all (are_connected ?from ?to))
            (over all (escorter_free ?r))
        )
        :effect (and 
            (at start (not (escorter_free_for_action ?r)))

            (at end (escorter_free_for_action ?r))
            (at end (not (at ?r ?from)))

            (at end (at ?r ?to))
        )
    )


    (:durative-action insert_item
        :parameters (?r - robot_carrier ?b - box ?i - item ?loc - location)
        :duration (= ?duration 1)
        :condition (and 
            (at start (is_empty ?b))
            (at start (carrier_free ?r))
            (at start (box_free ?b))
            (at start (carrier_free_for_action ?r))
            (at start (at ?r ?loc))
            (at start (at ?b ?loc))
            (at start (at_item ?i ?loc))

            (over all (at ?r ?loc))
            (over all (at ?b ?loc))
            (over all (at_item ?i ?loc))
        )
        :effect (and 
            ; make carrier busy and the box not free, box is for parallelism
            (at start (not (carrier_free_for_action ?r))) 
            (at start (not (carrier_free ?r)))
            (at start (not (box_free ?b)))

            (at end (not (is_empty ?b)))
            (at end (inside ?i ?b))
            (at end (box_free ?b))
            (at end (carrier_free ?r))
            (at end (carrier_free_for_action ?r))
        )
    )

    (:durative-action insert_box
        :parameters (?r - robot_carrier ?b - box ?c - container ?loc - location)
        :duration (= ?duration 2)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (box_free ?b))
            (at start (carrier_free ?r))
            (at start (container_free ?c))
            (at start (>= (container_capacity ?c) (+ (container_load ?c) 1)))
            (at start (at ?r ?loc))
            (at start (at ?b ?loc))
            (at start (at ?c ?loc))

            (over all (>= (container_capacity ?c) (+ (container_load ?c) 1)))
            (over all (at ?r ?loc))
            (over all (at ?b ?loc))
            (over all (at ?c ?loc))

        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))
            (at start (not (carrier_free ?r)))
            (at start (not (box_free ?b)))
            (at start (not (container_free ?c)))

            (at end (carrier_free_for_action ?r))
            (at end (carrier_free ?r))
            (at end (container_free ?c))
            (at end (increase (container_load ?c) 1))
            (at end (on ?b ?c))
        )
    )
    
    (:durative-action pickup_container
        :parameters (?r - robot_carrier ?c - container ?loc - location)
        :duration (= ?duration 3)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (carrier_free ?r))
            (at start (container_free ?c))
            (at start (at ?r ?loc))
            (at start (at ?c ?loc))

            (over all (at ?r ?loc))
            (over all (at ?c ?loc))

        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))
            (at start (not (carrier_free ?r)))
            (at start (not (container_free ?c)))

            (at end (carrier_free_for_action ?r))
            (at end (carrying_container ?r ?c))
        )
    )

    (:durative-action move_busy_carrier
        :parameters (?r - robot_carrier ?from - location ?to - location ?b - box ?c - container)
        :duration (= ?duration 4)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (carrying_container ?r ?c))
            (at start (on ?b ?c))
            (at start (are_connected ?from ?to))
            (at start (at ?r ?from))
            (at start (at ?c ?from))
            (at start (at ?b ?from))

            (over all (carrying_container ?r ?c))
            (over all (are_connected ?from ?to))
            (over all (on ?b ?c))

        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))

            (at end (carrier_free_for_action ?r))
            (at end (not (at ?r ?from)))
            (at end (not (at ?c ?from)))
            (at end (not (at ?b ?from)))

            (at end (at ?r ?to))
            (at end (at ?c ?to))
            (at end (at ?b ?to))
        )
    )

    (:durative-action deploy_container
        :parameters (?r - robot_carrier ?c - container ?loc - location)
        :duration (= ?duration 1)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (carrying_container ?r ?c))
            (at start (at ?r ?loc))
            (at start (at ?c ?loc))

            (over all (at ?r ?loc))
            (over all (at ?c ?loc))
        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))

            (at end (carrier_free_for_action ?r))
            (at end (carrier_free ?r))
            (at end (container_free ?c))
            (at end (not (carrying_container ?r ?c)))
        )
    )

    (:durative-action deploy_box
        :parameters (?r - robot_carrier ?c - container ?b - box ?loc - location)
        :duration (= ?duration 1)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (carrier_free ?r))
            (at start (container_free ?c))
            (at start (on ?b ?c))
            (at start (at ?r ?loc))
            (at start (at ?c ?loc))
            (at start (at ?b ?loc))

            (over all (at ?r ?loc))
            (over all (at ?c ?loc))
            (over all (at ?b ?loc))
            (over all (container_free ?c))
            (over all (carrier_free ?r))
        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))

            (at end (carrier_free_for_action ?r))
            (at end (box_free ?b))
            (at end (not (on ?b ?c)))
            (at end (decrease (container_load ?c) 1))
        )
    )

    (:durative-action deliver_item
        :parameters (?r - robot_carrier ?b - box ?loc - location ?i - item ?u - unit)
        :duration (= ?duration 1)
        :condition (and 
            (at start (carrier_free_for_action ?r))
            (at start (carrier_free ?r))
            (at start (need_item ?u ?i))
            ;(at start (not (has_item ?u ?i)))
            (at start (inside ?i ?b))
            (at start (at ?r ?loc))
            (at start (at ?u ?loc))
            (at start (at ?b ?loc))

            (over all (at ?r ?loc))
            (over all (at ?u ?loc))
            (over all (at ?b ?loc))
            (over all (box_free ?b))
            (over all (carrier_free ?r))
        )
        :effect (and 
            (at start (not (carrier_free_for_action ?r)))

            (at end (carrier_free_for_action ?r))
            (at end (not (need_item ?u ?i)))
            (at end (has_item ?u ?i))
            (at end (not (inside ?i ?b)))
            (at end (is_empty ?b))
        )
    )

)