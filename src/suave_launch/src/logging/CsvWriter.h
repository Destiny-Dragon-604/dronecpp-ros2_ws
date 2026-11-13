#ifndef CSVWRITER_H
#define CSVWRITER_H

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

// One CSV row, built with operator<<. Strings are quoted when they need it.
class CsvRow
{
public:
    CsvRow() { m_ss << std::setprecision(9); }

    template<typename T>
    CsvRow& operator<<(const T& value)
    {
        sep();
        m_ss << value;
        return *this;
    }

    CsvRow& operator<<(bool value)
    {
        sep();
        m_ss << (value ? 1 : 0);
        return *this;
    }

    CsvRow& operator<<(const std::string& value)
    {
        sep();
        if (value.find_first_of(",\"\n") == std::string::npos)
        {
            m_ss << value;
        }
        else
        {
            m_ss << '"';
            for (const char c : value)
            {
                if (c == '"') m_ss << '"';
                m_ss << (c == '\n' ? ' ' : c);
            }
            m_ss << '"';
        }
        return *this;
    }

    CsvRow& operator<<(const char* value) { return *this << std::string(value); }

    // Fixed-point with 6 decimals; used for unix time stamps (s) so they keep microseconds.
    CsvRow& time(double seconds)
    {
        sep();
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.6f", seconds);
        m_ss << buf;
        return *this;
    }

    // Writes exactly n cells from v, leaving missing ones empty.
    CsvRow& padded(const std::vector<float>& v, size_t n)
    {
        for (size_t i = 0; i < n; ++i)
        {
            sep();
            if (i < v.size()) m_ss << v[i];
        }
        return *this;
    }

    std::string str() const { return m_ss.str(); }

private:
    void sep()
    {
        if (!m_first) m_ss << ',';
        m_first = false;
    }

    std::ostringstream m_ss{};
    bool m_first{ true };
};

// Thread-safe append-only CSV file. Every row starts with t_unix (s) and t (s since the
// writer's clock origin) so all files of a flight line up.
class CsvWriter
{
public:
    using Clock = std::chrono::steady_clock;

    CsvWriter(const std::filesystem::path& path, const std::string& header, Clock::time_point origin) :
        m_path(path), m_origin(origin)
    {
        m_file.open(path, std::ofstream::out | std::ofstream::trunc);
        m_file << "t_unix,t," << header << '\n';
    }

    ~CsvWriter()
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        m_file.flush();
    }

    // Starts a row with the two time columns filled in.
    CsvRow begin() const
    {
        CsvRow row{};
        const double unix_s = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double t = std::chrono::duration<double>(Clock::now() - m_origin).count();
        row.time(unix_s).time(t);
        return row;
    }

    void write(const CsvRow& row)
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        m_file << row.str() << '\n';
        ++m_rows;
        // Flush once a second: a crash loses at most ~1 s, and callbacks stay cheap.
        const auto now = Clock::now();
        if (now - m_last_flush > std::chrono::seconds(1))
        {
            m_file.flush();
            m_last_flush = now;
        }
    }

    void flush()
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        m_file.flush();
    }

    size_t rows() const
    {
        std::lock_guard<std::mutex> lock{ m_mutex };
        return m_rows;
    }

    const std::filesystem::path& path() const { return m_path; }

private:
    std::filesystem::path m_path;
    Clock::time_point m_origin;
    mutable std::mutex m_mutex{};
    std::ofstream m_file{};
    size_t m_rows{ 0 };
    Clock::time_point m_last_flush{ Clock::now() };
};

#endif //CSVWRITER_H
