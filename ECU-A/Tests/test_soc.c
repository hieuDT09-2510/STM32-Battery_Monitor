/*
 * Unit test trên máy host cho bộ ước lượng SOC.
 * Bao phủ giá trị điện áp ban đầu, chiều sạc/xả, vùng chết (deadband), khoảng trống dữ liệu, tràn vòng Tick, giới hạn trên/dưới và yêu cầu lưu.
 */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_soc.h"

/* Đọc một bản snapshot SOC bắt buộc phải hợp lệ, giảm kiểm tra giá trị trả về lặp lại ở từng test. */
static SocSnapshot_t GetSoc(void)
{
    SocSnapshot_t snapshot;
    assert(Soc_GetSnapshot(&snapshot) == 1U);
    return snapshot;
}

/* Khi unit test SOC thông thường không quan tâm SOH, cung cấp một mức điện áp 3.60V hợp lệ. */
static void UpdateCurrent(int32_t current_uA,
                          uint32_t timestamp_ms)
{
    Soc_UpdateCurrent(current_uA, 3600U, 1U, timestamp_ms);
}

/*
 * Nạp liên tiếp các mẫu dòng điện không đổi cách đều nhau.
 * Lần gọi đầu tiên chỉ thiết lập điểm bắt đầu tích phân hình thang, các lần gọi sample_count tiếp theo mới thực sự tích phân.
 */
static void RunConstantCurrent(int32_t current_uA,
                               uint32_t start_ms,
                               uint32_t interval_ms,
                               uint32_t sample_count)
{
    uint32_t index;                    /* Số mẫu tích phân đã nạp. */
    uint32_t timestamp_ms = start_ms;  /* Dấu thời gian của mẫu hiện tại. */

    UpdateCurrent(current_uA, timestamp_ms);
    for (index = 0U; index < sample_count; index++)
    {
        timestamp_ms += interval_ms;
        UpdateCurrent(current_uA, timestamp_ms);
    }
}

static void TestVoltageInitialization(void)
{
    SocSnapshot_t snapshot; /* Dùng để kiểm tra ba điểm khởi tạo ban đầu: 3.05V, 3.60V và 4.15V. */

    Soc_InitEmpty();
    assert(Soc_GetSnapshot(&snapshot) == 0U);

    assert(Soc_TryInitializeFromVoltage(3050U, 10U) == 1U);
    snapshot = GetSoc();
    assert(snapshot.valid == 1U);
    assert(snapshot.init_source == SOC_INIT_SOURCE_VOLTAGE);
    assert(snapshot.soc_centi_percent == 0U);

    Soc_InitEmpty();
    assert(Soc_TryInitializeFromVoltage(3600U, 20U) == 1U);
    snapshot = GetSoc();
    assert(snapshot.soc_centi_percent == 5000U);

    Soc_InitEmpty();
    assert(Soc_TryInitializeFromVoltage(4150U, 30U) == 1U);
    snapshot = GetSoc();
    assert(snapshot.soc_centi_percent == 10000U);
}

static void TestPositiveCurrentDischarges(void)
{
    SocPersistentState_t stored; /* Tạo trạng thái khôi phục từ NVM đầy pin 100%. */
    SocSnapshot_t snapshot;      /* Kết quả sau khi xả 800mA trong 15 phút. */

    stored.remaining_uAs = SOC_FULL_CHARGE_UAS;
    stored.soc_centi_percent = 10000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    stored.timestamp_ms = 0U;

    Soc_InitFromStored(&stored, 0U);
    RunConstantCurrent(800000L, 0U, 100U, 9000U);
    snapshot = GetSoc();
    assert(snapshot.soc_centi_percent == 9000U);
    assert(snapshot.integration_count == 9000U);
}

static void TestNegativeCurrentCharges(void)
{
    SocPersistentState_t stored; /* Tạo điểm khởi đầu mức pin 50%. */
    SocSnapshot_t snapshot;      /* Kết quả sau khi sạc -800mA trong 15 phút. */

    stored.remaining_uAs = SOC_FULL_CHARGE_UAS / 2LL;
    stored.soc_centi_percent = 5000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    stored.timestamp_ms = 0U;

    Soc_InitFromStored(&stored, 0U);
    RunConstantCurrent(-800000L, 0U, 100U, 9000U);
    snapshot = GetSoc();
    assert(snapshot.soc_centi_percent == 6000U);
}

static void TestDeadbandAndGap(void)
{
    SocPersistentState_t stored; /* Tạo điểm khởi đầu mức pin 50%. */
    SocSnapshot_t before;        /* Snapshot đối chiếu trước thao tác. */
    SocSnapshot_t after;         /* Snapshot sau khi xử lý vùng chết hoặc khoảng trống quá thời gian. */

    stored.remaining_uAs = SOC_FULL_CHARGE_UAS / 2LL;
    stored.soc_centi_percent = 5000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    stored.timestamp_ms = 0U;

    Soc_InitFromStored(&stored, 0U);
    UpdateCurrent(1999L, 10U);
    before = GetSoc();
    UpdateCurrent(1999L, 110U);
    after = GetSoc();
    assert(after.remaining_uAs == before.remaining_uAs);

    UpdateCurrent(800000L, 111U);
    before = GetSoc();
    UpdateCurrent(800000L, 212U);
    after = GetSoc();
    assert(after.remaining_uAs == before.remaining_uAs);
    assert(after.gap_count == 1U);
}

static void TestTimestampWrapAndSaturation(void)
{
    SocPersistentState_t stored; /* Lần lượt tạo các trạng thái biên gần 0% và 100%. */
    SocSnapshot_t snapshot;      /* Kiểm tra tích phân khi tràn vòng và kẹp giới hạn trên/dưới. */

    stored.remaining_uAs = 1000LL;
    stored.soc_centi_percent = 0U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    stored.timestamp_ms = 0U;

    Soc_InitFromStored(&stored, 0U);
    UpdateCurrent(800000L, UINT32_MAX - 49U);
    UpdateCurrent(800000L, 50U);
    snapshot = GetSoc();
    assert(snapshot.gap_count == 0U);
    assert(snapshot.remaining_uAs == 0LL);
    assert(snapshot.soc_centi_percent == 0U);

    stored.remaining_uAs = SOC_FULL_CHARGE_UAS - 1000LL;
    stored.soc_centi_percent = 10000U;
    Soc_InitFromStored(&stored, 0U);
    UpdateCurrent(-800000L, 100U);
    UpdateCurrent(-800000L, 200U);
    snapshot = GetSoc();
    assert(snapshot.remaining_uAs == SOC_FULL_CHARGE_UAS);
    assert(snapshot.soc_centi_percent == 10000U);
}

static void TestPersistenceRequest(void)
{
    SocPersistentState_t stored;  /* Trạng thái 100% đã được lưu trữ gần nhất. */
    SocPersistentState_t request; /* Yêu cầu lưu được sao chép ra sau khi thỏa mãn chính sách. */

    stored.remaining_uAs = SOC_FULL_CHARGE_UAS;
    stored.soc_centi_percent = 10000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;
    stored.timestamp_ms = 0U;

    Soc_InitFromStored(&stored, 0U);
    Soc_MarkPersisted(&stored, 0U);
    assert(Soc_GetSaveRequest(&request) == 0U);

    RunConstantCurrent(800000L, 0U, 100U, 900U);
    assert(Soc_GetSaveRequest(&request) == 1U);
    assert(request.soc_centi_percent == 9900U);

    Soc_MarkPersisted(&request, 90000U);
    assert(Soc_GetSaveRequest(&request) == 0U);
}

static void TestAutomaticSohLearning(void)
{
    SocPersistentState_t stored;
    SocPersistentState_t request;
    SocSnapshot_t snapshot;
    uint32_t timestamp_ms;
    uint32_t index;

    (void)memset(&stored, 0, sizeof(stored));
    stored.remaining_uAs = SOC_FULL_CHARGE_UAS;
    stored.soc_centi_percent = 10000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;

    Soc_InitFromStored(&stored, 0U);
    Soc_MarkPersisted(&stored, 0U);

    /* Khi điều kiện sạc đầy duy trì liên tục 59.99 giây vẫn chỉ có thể chờ. */
    timestamp_ms = 0U;
    Soc_UpdateCurrent(0L, 4180U, 1U, timestamp_ms);
    for (index = 0U; index < 5999U; index++)
    {
        timestamp_ms += 10U;
        Soc_UpdateCurrent(0L, 4180U, 1U, timestamp_ms);
    }
    snapshot = GetSoc();
    assert(snapshot.soh_state == SOH_STATE_WAIT_FULL);
    assert(snapshot.soh_valid == 0U);

    /* Sau khi đạt 60 giây tự động vào trạng thái học, dung lượng lần này bắt đầu từ 0. */
    timestamp_ms += 10U;
    Soc_UpdateCurrent(0L, 4180U, 1U, timestamp_ms);
    snapshot = GetSoc();
    assert(snapshot.soh_state == SOH_STATE_LEARNING);
    assert(snapshot.soh_learning_uAs == 0LL);

    /* Xả 1A trong 360 giây; dòng dương bắt buộc phải được cộng dồn. */
    for (index = 0U; index < 3600U; index++)
    {
        timestamp_ms += 100U;
        Soc_UpdateCurrent(1000000L, 3800U, 1U, timestamp_ms);
    }
    snapshot = GetSoc();
    assert(snapshot.soh_learning_uAs > 359000000LL);
    assert(snapshot.soh_learning_uAs < 361000000LL);

    /* Điện áp thấp chưa đủ 5 giây thì chưa thể kết thúc. */
    for (index = 0U; index < 50U; index++)
    {
        timestamp_ms += 100U;
        Soc_UpdateCurrent(1000000L, 3000U, 1U, timestamp_ms);
    }
    snapshot = GetSoc();
    assert(snapshot.soh_state == SOH_STATE_LEARNING);

    /* Sau khi điện áp thấp đạt 5 giây sẽ hoàn tất, kết quả hợp lệ và bắt buộc yêu cầu lưu. */
    timestamp_ms += 100U;
    Soc_UpdateCurrent(1000000L, 3000U, 1U, timestamp_ms);
    snapshot = GetSoc();
    assert(snapshot.soh_state == SOH_STATE_WAIT_FULL);
    assert(snapshot.soh_valid == 1U);
    assert(snapshot.soh_capacity_uAh > 100000U);
    assert(snapshot.soh_capacity_uAh < 103000U);
    assert(snapshot.soh_centi_percent > 500U);
    assert(snapshot.soh_centi_percent < 520U);
    assert(Soc_GetSaveRequest(&request) == 1U);
    assert(request.soh_valid == 1U);
    assert(request.learned_capacity_uAh == snapshot.soh_capacity_uAh);
}

static void TestFullConfirmationCalibratesSoc(void)
{
    SocPersistentState_t stored;
    SocPersistentState_t request;
    SocSnapshot_t snapshot;
    uint32_t timestamp_ms = 0U;
    uint32_t index;

    (void)memset(&stored, 0, sizeof(stored));
    stored.remaining_uAs = SOC_FULL_CHARGE_UAS / 2LL;
    stored.soc_centi_percent = 5000U;
    stored.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;

    Soc_InitFromStored(&stored, 0U);
    Soc_MarkPersisted(&stored, 0U);

    /*
     * Mô-đun sạc hiện tại sau khi sạc đầy thì INA226 vẫn duy trì khoảng -170mA, do đó xác nhận sạc đầy
     * không thể dựa vào điều kiện dòng nhỏ mà chỉ có thể dùng điện áp cao duy trì liên tục.
     */
    Soc_UpdateCurrent(-170000L, 4180U, 1U, timestamp_ms);
    for (index = 0U; index < 6000U; index++)
    {
        timestamp_ms += 10U;
        Soc_UpdateCurrent(-170000L, 4180U, 1U, timestamp_ms);
    }

    snapshot = GetSoc();
    assert(snapshot.soh_state == SOH_STATE_LEARNING);
    assert(snapshot.remaining_uAs == SOC_FULL_CHARGE_UAS);
    assert(snapshot.soc_centi_percent == 10000U);
    assert(snapshot.save_pending == 1U);
    assert(Soc_GetSaveRequest(&request) == 1U);
    assert(request.remaining_uAs == SOC_FULL_CHARGE_UAS);
    assert(request.soc_centi_percent == 10000U);
}

static void TestSohSaveRequestSurvivesOlderWrite(void)
{
    SocPersistentState_t old_saved;
    SocPersistentState_t pending;
    uint32_t timestamp_ms = 0U;
    uint32_t index;

    (void)memset(&old_saved, 0, sizeof(old_saved));
    old_saved.remaining_uAs = SOC_FULL_CHARGE_UAS;
    old_saved.soc_centi_percent = 10000U;
    old_saved.rated_capacity_uAh = SOC_RATED_CAPACITY_UAH;

    Soc_InitFromStored(&old_saved, 0U);
    Soc_MarkPersisted(&old_saved, 0U);

    Soc_UpdateCurrent(0L, 4180U, 1U, timestamp_ms);
    for (index = 0U; index < 6000U; index++)
    {
        timestamp_ms += 10U;
        Soc_UpdateCurrent(0L, 4180U, 1U, timestamp_ms);
    }

    for (index = 0U; index < 51U; index++)
    {
        timestamp_ms += 100U;
        Soc_UpdateCurrent(0L, 3000U, 1U, timestamp_ms);
    }
    assert(Soc_GetSaveRequest(&pending) == 1U);
    assert(pending.soh_valid == 1U);

    /* Giả lập lúc này Flash mới xác nhận bản sao cũ hơn chưa có SOH đã ghi thành công. */
    Soc_MarkPersisted(&old_saved, timestamp_ms);
    assert(Soc_GetSaveRequest(&pending) == 1U);
    assert(pending.soh_valid == 1U);
}

int main(void)
{
    TestVoltageInitialization();
    TestPositiveCurrentDischarges();
    TestNegativeCurrentCharges();
    TestDeadbandAndGap();
    TestTimestampWrapAndSaturation();
    TestPersistenceRequest();
    TestFullConfirmationCalibratesSoc();
    TestAutomaticSohLearning();
    TestSohSaveRequestSurvivesOlderWrite();
    puts("SOC estimator tests passed");
    return 0;
}
