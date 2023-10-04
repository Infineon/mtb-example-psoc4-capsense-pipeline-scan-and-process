/******************************************************************************
 * File Name: main.c
 *
 * Description: This is the source code for the PSoC 4: CAPSENSE Pipeline
 * scan and process code example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *******************************************************************************
 * Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
#ifdef COMPONENT_PSOC4100SMAX
#define CAPSENSE_MSC0_INTR_PRIORITY      (3u)
#define CAPSENSE_MSC1_INTR_PRIORITY      (3u)
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
#define CAPSENSE_INTR_PRIORITY           (3u)
#endif

#define CY_ASSERT_FAILED                 (0u)

/* EZI2C interrupt priority must be higher than CapSense interrupt */
#define EZI2C_INTR_PRIORITY              (2u)

#ifdef COMPONENT_PSOC4100SMAX
#define NUMBER_OF_SLIDER_SEGMENTS        (8u)
#define NUMBER_OF_BUTTONS                (2u)

#if CY_CAPSENSE_BIST_EN
#define SHIELD_MEAS_SKIP_CH_MASK         (0u)
#endif

#elif COMPONENT_PSOC4100SP
#define NUMBER_OF_SLIDER_SEGMENTS        (6u)
#define NUMBER_OF_BUTTONS                (3u)
#else /* COMPONENT_PSOC4000S */
#define NUMBER_OF_SLIDER_SEGMENTS        (5u)
#define NUMBER_OF_BUTTONS                (3u)
#endif

/* ILO Frequency in Hz */
#define ILO_FREQUENCY_HZ                 (40000U)

/* WDT interrupt period in milliseconds. Max limit is 1698 ms */
#define WDT_INTERRUPT_INTERVAL_MS        (10U)

/* WDT interrupt priority */
#define WDT_INTERRUPT_PRIORITY           (3u)

/* Define desired delay in microseconds */
#define DESIRED_WDT_INTERVAL             (WDT_INTERRUPT_INTERVAL_MS  * 1000U)

/* LED STATE TOGGLE MACROS */
#define LED_STATE_ON    (0u)
#define LED_STATE_OFF   (1u)

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/
cy_stc_scb_ezi2c_context_t ezi2c_context;

#if CY_CAPSENSE_BIST_EN
/* Variables to hold sensor parasitic capacitances */
uint32_t sensor_cp0 = 0;
uint32_t sensor_cp1 = 0;
uint32_t sensor_cp2 = 0;
uint32_t sense_cap[NUMBER_OF_SLIDER_SEGMENTS] = {0};
uint32_t shield_cap;
cy_en_capsense_bist_status_t status,measure_status[NUMBER_OF_SLIDER_SEGMENTS];
cy_en_capsense_bist_status_t shield_meas_status;
#endif /* CY_CAPSENSE_BIST_EN */

#ifdef COMPONENT_PSOC4100SMAX
/* Variables to hold LED Port and Pins for Buttons */
GPIO_PRT_Type * LED_PORT_BUTT[NUMBER_OF_BUTTONS] = {P6_4_PORT, P12_0_PORT};
uint8_t LED_PIN_BUTT[NUMBER_OF_BUTTONS] = {P6_4_PIN, P12_0_PIN};

#elif COMPONENT_PSOC4100SP
/* Variables to hold LED Port and Pins for Buttons and Sliders */
GPIO_PRT_Type * LED_PORT_BUTT[NUMBER_OF_BUTTONS] = {P5_2_PORT, P5_5_PORT, P5_7_PORT};
uint8_t LED_PIN_BUTT[NUMBER_OF_BUTTONS] = {P5_2_PIN, P5_5_PIN, P5_7_PIN};

GPIO_PRT_Type * LED_PORT_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
{P1_0_PORT,P1_2_PORT,P1_4_PORT,P1_6_PORT,P2_0_PORT,P2_2_PORT};
uint8_t LED_PIN_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
{P1_0_PIN,P1_2_PIN,P1_4_PIN,P1_6_PIN,P2_0_PIN,P2_2_PIN};

#else /* COMPONENT_PSOC4000S */
/* Variables to hold LED Port and Pins for Buttons and Sliders */
GPIO_PRT_Type * LED_PORT_BUTT[NUMBER_OF_BUTTONS] = {P3_4_PORT, P3_5_PORT, P3_6_PORT};
uint8_t LED_PIN_BUTT[NUMBER_OF_BUTTONS] = {P3_4_PIN, P3_5_PIN, P3_6_PIN};

GPIO_PRT_Type * LED_PORT_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
{P2_0_PORT,P2_1_PORT,P2_2_PORT,P2_3_PORT,P2_4_PORT};
uint8_t LED_PIN_SLIDER[NUMBER_OF_SLIDER_SEGMENTS] =
{P2_0_PIN,P2_1_PIN,P2_2_PIN,P2_3_PIN,P2_4_PIN};
#endif

/* WDT interrupt service routine configuration */
const cy_stc_sysint_t wdt_isr_cfg =
{
    .intrSrc = srss_interrupt_wdt_IRQn,    /* Interrupt source */
    .intrPriority = WDT_INTERRUPT_PRIORITY /* Interrupt priority is 3 */
};

/* Variable to check whether WDT interrupt is triggered */
volatile bool interrupt_flag = false;

/* Variable to store the counts required after ILO compensation */
static uint32_t ilo_compensated_counts = 0U;
static uint32_t ilo_cycles  = 0U;


/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/* WDT interrupt service routine */
void wdt_isr(void);
/* WDT function */
void wdt_trigger(void);

static void initialize_capsense(void);
#ifdef COMPONENT_PSOC4100SMAX
static void capsense_msc0_isr(void);
static void capsense_msc1_isr(void);
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
static void capsense_isr(void);
#endif
static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);
static void led_control(uint8_t WidgetID);

#if CY_CAPSENSE_BIST_EN
static void measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CapSense
 *  - initialize tuner communication
 *  - perform Cp measurement if Built-in Self test (BIST) is enabled
 *  - scan touch input continuously
 *
 * Return:
 *  int
 *
 * Parameters:
 *  void
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize and enable interrupt */
    cy_en_sysint_status_t sysintStatus = Cy_SysInt_Init(&wdt_isr_cfg, wdt_isr);
    if(sysintStatus != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }
    NVIC_EnableIRQ(wdt_isr_cfg.intrSrc);

    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* This function initializes the WDT block */
    Cy_WDT_Init();

    /* Enable the ILO */
    Cy_SysClk_IloEnable();

    /* Now switch the WDC timers clocking to ILO */
    /* Disable the WCO */
    Cy_SysClk_WcoDisable();

    /* Enable WDT */
    Cy_WDT_Enable();

    /* Unmask the WDT interrupt */
    Cy_WDT_UnmaskInterrupt();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    /* Initialize CapSense */
    initialize_capsense();

    uint8_t currentWidgetID, previousWidgetID;

    currentWidgetID = 0u;
    previousWidgetID = 0u;
    uint8_t numWgt = cy_capsense_context.ptrCommonConfig->numWd;

    cy_stc_syspm_callback_params_t ezi2cCallbackParams =
    {
        .base       = CYBSP_EZI2C_HW,
        .context    = &ezi2c_context
    };

    cy_stc_syspm_callback_t ezi2cCallback =
    {
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
    };

#ifdef COMPONENT_PSOC4100SMAX

    cy_stc_syspm_callback_params_t sysClkCallbackParams =
    {
        .base       = CYBSP_MSC0_HW,
        .context    = &cy_capsense_context
    };

    cy_stc_syspm_callback_params_t sysClkCallbackParams1 =
    {
        .base       = CYBSP_MSC1_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &Cy_SysClk_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };
    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback1 =
    {
        .callback       = &Cy_SysClk_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams1,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };

    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);
    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback1);

#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
    cy_stc_syspm_callback_params_t sysClkCallbackParams =
    {
        .base       = CYBSP_CSD_HW,
        .context    = &cy_capsense_context
    };

    /* Callback declaration for Deep Sleep mode */
    cy_stc_syspm_callback_t sysClkCallback =
    {
        .callback       = &Cy_SysClk_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &sysClkCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 1
    };

    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&sysClkCallback);
#endif

#ifdef COMPONENT_PSOC4100SMAX
    /* Start the first scan of Previous Widget */
    Cy_CapSense_ScanSlots(cy_capsense_context.ptrWdConfig[previousWidgetID].firstSlotId,
            cy_capsense_context.ptrWdConfig[previousWidgetID].numSlots, &cy_capsense_context);
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
    /* Start the first scan of Previous Widget */
    Cy_CapSense_ScanWidget(previousWidgetID,&cy_capsense_context);
#endif

    for (;;)
    {
        wdt_trigger();

        if(CY_CAPSENSE_NOT_BUSY == Cy_CapSense_IsBusy(&cy_capsense_context))
        {
            /* Increase the currentWidgetID by 1 i.e, point to the next Widget */
            currentWidgetID = (currentWidgetID < (numWgt - 1u))?(currentWidgetID + 1u):0;

#ifdef COMPONENT_PSOC4100SMAX
            Cy_CapSense_ScanSlots(cy_capsense_context.ptrWdConfig[currentWidgetID].firstSlotId,
                    cy_capsense_context.ptrWdConfig[currentWidgetID].numSlots, &cy_capsense_context);
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
            /* Start the first scan of Current Widget */
            Cy_CapSense_ScanWidget(currentWidgetID,&cy_capsense_context);
#endif

            /* Process the Previous Widget */
            Cy_CapSense_ProcessWidget(previousWidgetID,&cy_capsense_context);

            /* Turning ON/OFF based on widget status */
            led_control(previousWidgetID);

            /* Set the previous widget as current widget */
            previousWidgetID = currentWidgetID;

            /* Establishes synchronized communication with the CapSense Tuner tool */
            Cy_CapSense_RunTuner(&cy_capsense_context);

#if CY_CAPSENSE_BIST_EN
            /* Measure the self capacitance of sensor electrode using BIST */
            measure_sensor_cp();
#endif /* CY_CAPSENSE_BIST_EN */

        }
    }
}

/*******************************************************************************
 * Function Name: wdt_trigger
 ********************************************************************************
 * Summary:
 *  - Updates the set match value to the WDT block.
 *  - Enters into deep sleep mode.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
void wdt_trigger(void)
{
    if (interrupt_flag)
    {
        /* Clear the interrupt flag */
        interrupt_flag = false;

        /* Update the match count  */
        Cy_WDT_SetMatch((uint16_t)(ilo_compensated_counts + Cy_WDT_GetMatch()));
    }

    /* Start ILO measurement */
    Cy_SysClk_IloStartMeasurement();

    /* Get the ILO compensated counts i.e. the actual counts for the
     desired ILO frequency. ILO default accuracy is +/- 60%.
     Note that DESIRED_WDT_INTERVAL should be less than the total
     count time */
    while (CY_SYSCLK_SUCCESS != Cy_SysClk_IloCompensate(DESIRED_WDT_INTERVAL, &ilo_cycles));
    ilo_compensated_counts = (uint32_t)ilo_cycles;
    /* Stop ILO measurement before entering deep sleep mode */
    Cy_SysClk_IloStopMeasurement();
    /* Enter deep sleep mode */
    Cy_SysPm_CpuEnterDeepSleep();
}


/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CapSense and configures the CapSense
 *  interrupt.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

#ifdef COMPONENT_PSOC4100SMAX

    /* CapSense interrupt configuration MSC 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
        .intrSrc = CY_MSC0_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* CapSense interrupt configuration MSC 1 */
    const cy_stc_sysint_t capsense_msc1_interrupt_config =
    {
        .intrSrc = CY_MSC1_IRQ,
        .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */

    /* CapSense interrupt configuration */
    const cy_stc_sysint_t capsense_interrupt_config =
    {
        .intrSrc = CYBSP_CSD_IRQ,
        .intrPriority = CAPSENSE_INTR_PRIORITY,
    };
#endif

    /* Capture the CSD HW block and initialize it to the default state */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {

#ifdef COMPONENT_PSOC4100SMAX
        /* Initialize CapSense interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        /* Initialize CapSense interrupt for MSC 1 */
        Cy_SysInt_Init(&capsense_msc1_interrupt_config, capsense_msc1_isr);
        NVIC_ClearPendingIRQ(capsense_msc1_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc1_interrupt_config.intrSrc);
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
        /* Initialize CapSense interrupt */
        Cy_SysInt_Init(&capsense_interrupt_config, capsense_isr);
        NVIC_ClearPendingIRQ(capsense_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_interrupt_config.intrSrc);
#endif

        /* Initialize the CapSense firmware modules */
        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CapSense sensors are tuned
         * as per procedure give in the Readme.md file
         */
        CY_ASSERT(CY_ASSERT_FAILED);
    }
}

/*****************************************************************************
 * Function Name: wdt_isr
 ******************************************************************************
 * Summary:
 * This function is the handler for the WDT interrupt
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *****************************************************************************/
void wdt_isr(void)
{
    /* Clears the WDT match flag */
    Cy_WDT_ClearInterrupt();
    /* Set the interrupt flag */
    interrupt_flag = true;
}

#ifdef COMPONENT_PSOC4100SMAX
/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: capsense_msc1_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense MSC1 block.
 *
 *******************************************************************************/
static void capsense_msc1_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSC1_HW, &cy_capsense_context);
}

#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
/*******************************************************************************
 * Function Name: capsense_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CapSense block.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void capsense_isr(void)
{
    Cy_CapSense_InterruptHandler(CYBSP_CSD_HW, &cy_capsense_context);
}
#endif

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CapSense Tuner tool.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
        .intrSrc = CYBSP_EZI2C_IRQ,
        .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CapSense data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);

    /* Enables the SCB block for the EZI2C operation */
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

}


#ifdef COMPONENT_PSOC4100SMAX
/*******************************************************************************
 * Function Name: led_control
 ********************************************************************************
 * Summary:
 *  Toggle the LEDs based on the Active status of CapSense Widgets and their respective Sensors
 *
 * Return:
 *  void
 *
 * Parameters:
 *  WidgetID
 *******************************************************************************/
static void led_control(uint8_t WidgetID)
{
    if(0 != Cy_CapSense_IsWidgetActive(WidgetID, &cy_capsense_context))
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {
            if(0 != Cy_CapSense_IsWidgetActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID, &cy_capsense_context))
            {
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_STATE_ON);
            }
            else
            {
                Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_STATE_OFF);
            }
        }
        else{

            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_ON);
        }
    }
    else
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {

            Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, LED_STATE_OFF);
        }
        else{

            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_OFF);
        }
    }
}
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
static void led_control(uint8_t WidgetID)
{
    if(0 != Cy_CapSense_IsWidgetActive(WidgetID, &cy_capsense_context))
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {
            for(uint8_t sens_id = 0 ; sens_id < NUMBER_OF_SLIDER_SEGMENTS; sens_id++)
            {
                if(0 != Cy_CapSense_IsSensorActive(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,sens_id,&cy_capsense_context))
                {
                    Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_ON);
                }
                else
                {
                    Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_OFF);
                }
            }
        }
        else{
            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_ON);
        }
    }
    else
    {
        if(WidgetID == CY_CAPSENSE_LINEARSLIDER0_WDGT_ID)
        {
            for(uint8_t sens_id = 0 ; sens_id < NUMBER_OF_SLIDER_SEGMENTS; sens_id++)
            {
                Cy_GPIO_Write(LED_PORT_SLIDER[sens_id], LED_PIN_SLIDER[sens_id], LED_STATE_OFF);
            }
        }
        else{
            Cy_GPIO_Write(LED_PORT_BUTT[WidgetID], LED_PIN_BUTT[WidgetID], LED_STATE_OFF);
        }
    }
}
#endif

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}


#if CY_CAPSENSE_BIST_EN
/*******************************************************************************
 * Function Name: measure_sensor_cp
 ********************************************************************************
 * Summary:
 *  Measures the self capacitance of the sensor electrode (Cp) in Femto Farad and
 *  stores its value in the variable sensor_cp.
 *
 * Return:
 *  void
 *
 * Parameters:
 *  void
 *******************************************************************************/
#ifdef COMPONENT_PSOC4100SMAX

static void measure_sensor_cp()
{
    /* Measure the self capacitance of sensor electrode */
    status = Cy_CapSense_MeasureCapacitanceSensorElectrode(CY_CAPSENSE_BUTTON0_WDGT_ID,
            CY_CAPSENSE_BUTTON0_SNS0_ID, &cy_capsense_context);
    sensor_cp0 = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON0_WDGT_ID].ptrEltdCapacitance[CY_CAPSENSE_BUTTON0_SNS0_ID];

    status = Cy_CapSense_MeasureCapacitanceSensorElectrode(CY_CAPSENSE_BUTTON1_WDGT_ID,
            CY_CAPSENSE_BUTTON1_SNS0_ID, &cy_capsense_context);
    sensor_cp1 = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_BUTTON1_WDGT_ID].ptrEltdCapacitance[CY_CAPSENSE_BUTTON1_SNS0_ID];

    for(uint8_t sensor_id = CY_CAPSENSE_LINEARSLIDER0_SNS0_ID; sensor_id < NUMBER_OF_SLIDER_SEGMENTS; sensor_id++)
    {
        measure_status[sensor_id] =
                Cy_CapSense_MeasureCapacitanceSensorElectrode(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                    sensor_id, &cy_capsense_context);
        sense_cap[sensor_id] = cy_capsense_context.ptrWdConfig[CY_CAPSENSE_LINEARSLIDER0_WDGT_ID].ptrEltdCapacitance[sensor_id];
    }
    Cy_CapSense_SetInactiveElectrodeState(CY_CAPSENSE_SNS_CONNECTION_SHIELD,
            CY_CAPSENSE_BIST_SHIELD_GROUP, &cy_capsense_context);
    shield_meas_status = Cy_CapSense_MeasureCapacitanceShieldElectrode(SHIELD_MEAS_SKIP_CH_MASK,
        &cy_capsense_context);
    shield_cap = cy_capsense_context.ptrBistContext->ptrChShieldCap[0];
}
#else /* COMPONENT_PSOC4100SP, COMPONENT_PSOC4000S */
static void measure_sensor_cp()
{
    /* Measure the self capacitance of sensor electrode */
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON0_WDGT_ID,
            CY_CAPSENSE_BUTTON0_SNS0_ID, &sensor_cp0, &cy_capsense_context);
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON1_WDGT_ID,
            CY_CAPSENSE_BUTTON1_SNS0_ID, &sensor_cp1, &cy_capsense_context);
    status = Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_BUTTON2_WDGT_ID,
            CY_CAPSENSE_BUTTON2_SNS0_ID, &sensor_cp2, &cy_capsense_context);
    for(uint8_t sensor_id = CY_CAPSENSE_LINEARSLIDER0_SNS0_ID; sensor_id < CY_CAPSENSE_NUM_SNS_VALUE; sensor_id++){
        measure_status[sensor_id] =
                Cy_CapSense_MeasureCapacitanceSensor(CY_CAPSENSE_LINEARSLIDER0_WDGT_ID,
                    sensor_id,&sense_cap[sensor_id], &cy_capsense_context);
    }
}
#endif

#endif /* CY_CAPSENSE_BIST_EN */


/* [] END OF FILE */
