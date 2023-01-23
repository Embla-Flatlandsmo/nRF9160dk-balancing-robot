cmake_minimum_required(VERSION 3.16.0)

if(CONFIG_QDEC_PCA63539)

    target_sources(app PRIVATE
        qdec_pca63539.c
    )

endif()