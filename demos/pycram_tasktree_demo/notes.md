* demo_bullet.py
    * introspection_demo
        * Perform ActionDesignator: park arms
        * Perform ActionDesignator: Navigate to some target
        * Perform ActionDesignator: Open drawer
        * Perform ActionDesignator: park arms
        * Try to pick up bowl
            * Perform ActionDesignator: Look at bowl
            * Perform ActionDesignator: Detect bowl
            * Perform ActionDesignator: Pick up bowl
            * Iterate over possible navigation targets until pickup succeeds
        
    * prospection_demo
        * Perform ActionDesignator: park arms
        * Perform ActionDesignator: Navigate to some target
        * With simulated task tree:
            * Try to open drawer, pick up spoon from inside
        * Then use the generated parameters to parameterize another ActionDesignator, this time executed IRL
    
    * set_table
        * Complex demo:
            * Fetch bowl
            * Deliver bowl
            * Fetch spoon
            * Deliver spoon
            * Transport milk
            * Transport cereal
    

    