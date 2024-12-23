(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)
  
  (:types
    robot location section object - object
    hand - object
  )

  (:predicates
    ; Robot location predicates
    (at ?r - robot ?l - location)
    (holding ?h - hand ?o - object) ; New Predicate
    (hand-at ?h - hand ?s - section)
    (section-of ?s - section ?l - location)
    (free-hand ?h - hand)
    (connected ?l1 - location ?l2 - location)
    
    ; Accessibility predicates
    (left-accessible ?s - section)
    (right-accessible ?s - section)
  )

  ; Action: Move the robot from one location to another
  (:action move
    :parameters (?rob - robot ?from - location ?to - location)
    :precondition (and
                    (at ?rob ?from)
                    (connected ?from ?to))
    :effect (and
              (not (at ?rob ?from))
              (at ?rob ?to))
  )

  ; Action: Move a hand to a specific section
  (:action approach_section
    :parameters (?r - robot ?h - hand ?l - location ?s - section)
    :precondition (and
                    (at ?r ?l)
                    (free-hand ?h)
                    (section-of ?s ?l))
    :effect (hand-at ?h ?s)
  )

  ; Action: Retract a hand from a section
  (:action retract-hand
    :parameters (?h - hand ?s - section)
    :precondition (hand-at ?h ?s)
    :effect (and
              (not (hand-at ?h ?s))
              (free-hand ?h))
  )

  ; Action: Pick an object with a hand
  (:action pick
    :parameters (?r - robot 
                ?h - hand 
                ?o - object 
                ?s - section)
    :precondition (and
                    (free-hand ?h)
                    (hand-at ?h ?s)
                    (at ?o ?s)) ; Ensure object is at the section
    :effect (and
              (not (free-hand ?h))
              (holding ?h ?o)
              (not (at ?o ?s))) ; Object is now held, not at the section
  )

  ; Action: Release an object from a hand
  (:action release
    :parameters (?r - robot 
                ?h - hand 
                ?o - object 
                ?s - section)
    :precondition (and
                    (holding ?h ?o)
                    (hand-at ?h ?s))
    :effect (and
              (free-hand ?h)
              (not (holding ?h ?o))
              (at ?o ?s)) ; Object is placed back at the section
  )

  ; New Action: Approach both hands to different sections simultaneously
  (:action approachsections2arms
    :parameters (?r - robot 
                ?h1 - hand 
                ?h2 - hand 
                ?l - location 
                ?s1 - section 
                ?s2 - section)
    :precondition (and
                    (at ?r ?l)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (section-of ?s1 ?l)
                    (section-of ?s2 ?l)
                    (not (= ?h1 ?h2))
                    (not (= ?s1 ?s2))
                    
                    ; Accessibility constraints for each hand-section pair
                    (or (and (= ?h1 LEFT-HAND) (left-accessible ?s1))
                        (and (= ?h1 RIGHT-HAND) (right-accessible ?s1)))
                    (or (and (= ?h2 LEFT-HAND) (left-accessible ?s2))
                        (and (= ?h2 RIGHT-HAND) (right-accessible ?s2)))
                  )
    :effect (and
              (hand-at ?h1 ?s1)
              (hand-at ?h2 ?s2))
  )
)
