(define (domain robot_bartender)
  (:requirements :strips :typing :equality :adl)

  (:types
    robot location section object - object
    glass - object
    hand - object
  )

  (:predicates
    (at ?r - robot ?l - location)
    (section-of ?s - section ?l - location)
    (holding ?h - hand ?g - glass)
    (free-hand ?h - hand)
    (in-section ?g - glass ?s - section)
    (drink-prepared ?g - glass)
    (connected ?l1 - location ?l2 - location)  ; Connectivity between locations
  )

  ; Action: Move the robot from one location to another
  (:action move
    :parameters (?rob - robot ?from - location ?to - location)
    :precondition (and
                    (at ?rob ?from)
                    (connected ?from ?to))   ; Robot can only move between connected locations
    :effect (and
              (not (at ?rob ?from))
              (at ?rob ?to))
  )

  ; Action: Pick up two glasses simultaneously with both hands
  (:action pick-up-two-glasses
    :parameters (?r - robot
                ?h1 - hand ?g1 - glass ?s1 - section
                ?h2 - hand ?g2 - glass ?s2 - section
                ?l - location)
    :precondition (and
                    (at ?r ?l)
                    (section-of ?s1 ?l)
                    (section-of ?s2 ?l)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (not (= ?g1 ?g2))
                    (not (= ?h1 ?h2)))
    :effect (and
              (holding ?h1 ?g1)
              (holding ?h2 ?g2)
              (not (free-hand ?h1))
              (not (free-hand ?h2))
              (not (in-section ?g1 ?s1))
              (not (in-section ?g2 ?s2)))
  )

  ; Action: Pick up a single glass with one hand
  ; Only allowed if the other hand is occupied
  (:action pick-up-glass
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?l - location ?oh - hand)
    :precondition (and
                    (at ?r ?l)
                    (section-of ?s ?l)
                    (in-section ?g ?s)
                    (free-hand ?h)
                    (not (= ?h ?oh))
                    (not (free-hand ?oh))) ; Other hand is occupied
    :effect (and
              (holding ?h ?g)
              (not (free-hand ?h))
              (not (in-section ?g ?s)))
  )

  ; Action: Place two glasses from both hands into sections
  (:action place-two-glasses
    :parameters (?r - robot
                ?h1 - hand ?g1 - glass ?s1 - section
                ?h2 - hand ?g2 - glass ?s2 - section
                ?l - location)
    :precondition (and
                    (holding ?h1 ?g1)
                    (holding ?h2 ?g2)
                    (at ?r ?l)
                    (section-of ?s1 ?l)
                    (section-of ?s2 ?l)
                    (not (= ?h1 ?h2))
                    (not (= ?g1 ?g2))
                    (or
                      (= ?l preparation-table)
                      (and
                        (drink-prepared ?g1)
                        (drink-prepared ?g2))))
    :effect (and
              (free-hand ?h1)
              (free-hand ?h2)
              (not (holding ?h1 ?g1))
              (not (holding ?h2 ?g2))
              (in-section ?g1 ?s1)
              (in-section ?g2 ?s2))
  )

  ; Action: Place a glass from a hand into a section
  ; At the preparation table, can place any glass
  (:action place-glass
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?l - location)
    :precondition (and
                    (holding ?h ?g)
                    (at ?r ?l)
                    (section-of ?s ?l)
                    (or
                      (= ?l preparation-table)
                      (drink-prepared ?g))) ; At customer tables, glass must be prepared
    :effect (and
              (free-hand ?h)
              (not (holding ?h ?g))
              (in-section ?g ?s))
  )

  ; Action: Prepare a drink for a glass on the preparation table
  (:action prepare-drink-on-table
    :parameters (?r - robot ?g - glass ?s - section)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s preparation-table)
                    (in-section ?g ?s)
                    (not (drink-prepared ?g)))
    :effect (drink-prepared ?g)
  )

  ; Action: Prepare two drinks on the preparation table
  (:action prepare-two-drinks-on-table
    :parameters (?r - robot
                    ?g1 - glass ?s1 - section
                    ?g2 - glass ?s2 - section)
    :precondition (and
                    (at ?r preparation-table)
                    (free-hand left-hand)
                    (free-hand right-hand)
                    (section-of ?s1 preparation-table)
                    (section-of ?s2 preparation-table)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (not (drink-prepared ?g1))
                    (not (drink-prepared ?g2))
                    (not (= ?g1 ?g2)))
    :effect (and
              (drink-prepared ?g1)
              (drink-prepared ?g2))
  )

  ; Action: Pick up two prepared drinks from the preparation table
  (:action pick-up-two-drinks
    :parameters (?r - robot
                    ?h1 - hand ?g1 - glass ?s1 - section
                    ?h2 - hand ?g2 - glass ?s2 - section)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s1 preparation-table)
                    (section-of ?s2 preparation-table)
                    (in-section ?g1 ?s1)
                    (in-section ?g2 ?s2)
                    (free-hand ?h1)
                    (free-hand ?h2)
                    (drink-prepared ?g1)
                    (drink-prepared ?g2)
                    (not (= ?g1 ?g2))
                    (not (= ?h1 ?h2)))
    :effect (and
              (holding ?h1 ?g1)
              (holding ?h2 ?g2)
              (not (free-hand ?h1))
              (not (free-hand ?h2))
              (not (in-section ?g1 ?s1))
              (not (in-section ?g2 ?s2)))
  )

  ; Action: Pick up a single prepared drink from the preparation table
  ; Only allowed if the other hand is occupied
  (:action pick-up-drink
    :parameters (?r - robot ?h - hand ?g - glass ?s - section ?oh - hand)
    :precondition (and
                    (at ?r preparation-table)
                    (section-of ?s preparation-table)
                    (in-section ?g ?s)
                    (free-hand ?h)
                    (not (= ?h ?oh))
                    (not (free-hand ?oh)) ; Other hand is occupied
                    (drink-prepared ?g))
    :effect (and
              (holding ?h ?g)
              (not (free-hand ?h))
              (not (in-section ?g ?s)))
  )
)
